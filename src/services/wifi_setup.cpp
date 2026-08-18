#include "services/wifi_setup.h"

#include <WiFi.h>
#include <WiFiManager.h>

#include <cstdio>

#include <Preferences.h>
#include <esp_system.h>
#include <esp_wifi.h>

#ifdef WM_MDNS
#include <ESPmDNS.h>
#endif

#include "config.h"
#include "services/radar_location.h"
#include "ui/radar_range.h"
#include "ui/status_screens.h"

portMUX_TYPE s_boot_mux = portMUX_INITIALIZER_UNLOCKED;
volatile bool s_boot_tap_pending = false;
volatile bool s_boot_is_down = false;
volatile unsigned long s_boot_down_ms = 0;
bool s_long_press_handled = false;
bool s_boot_interrupt_attached = false;

void IRAM_ATTR onBootButtonIsr() {
  const bool down = digitalRead(config::kBootPin) == LOW;
  const unsigned long now = millis();
  portENTER_CRITICAL_ISR(&s_boot_mux);
  if (down) {
    s_boot_is_down = true;
    s_boot_down_ms = now;
  } else if (s_boot_is_down) {
    const unsigned long held = now - s_boot_down_ms;
    if (held >= config::kBootTapMinMs && held < config::kBootResetHoldMs) {
      s_boot_tap_pending = true;
    }
    s_boot_is_down = false;
  }
  portEXIT_CRITICAL_ISR(&s_boot_mux);
}

void initBootButton() {
  pinMode(config::kBootPin, INPUT_PULLUP);
  if (s_boot_interrupt_attached) {
    return;
  }
  attachInterrupt(digitalPinToInterrupt(static_cast<uint8_t>(config::kBootPin)),
                  onBootButtonIsr, CHANGE);
  s_boot_interrupt_attached = true;
}

namespace {

/** Separate from planeradar prefs (rangeInit) to avoid NVS handle conflicts. */
constexpr char kWifiPrefsNamespace[] = "wifi";
constexpr char kPrefsForcePortalKey[] = "portal";

bool s_force_config_portal = false;
WiFiManager s_wm;
bool s_wm_configured = false;

constexpr size_t kMaxSavedWifiNetworks = 4;

struct SavedWifiProfile {
  String ssid;
  String pass;
  double lat = config::kDefaultRadarLat;
  double lon = config::kDefaultRadarLon;
  bool has_location = false;
  uint32_t priority = 0;
};

void ensureWifiManager();
void startLanWebPortal();
void stopLanWebPortal();
bool wifiLinkUp();

String wifiProfileKey(size_t index, const char* suffix) {
  char key[32];
  snprintf(key, sizeof(key), "net%zu_%s", index, suffix);
  return String(key);
}

void removeSavedWifiProfileKeys(Preferences& prefs) {
  for (size_t index = 0; index < kMaxSavedWifiNetworks; ++index) {
    prefs.remove(wifiProfileKey(index, "ssid").c_str());
    prefs.remove(wifiProfileKey(index, "pass").c_str());
    prefs.remove(wifiProfileKey(index, "lat").c_str());
    prefs.remove(wifiProfileKey(index, "lon").c_str());
    prefs.remove(wifiProfileKey(index, "has_loc").c_str());
    prefs.remove(wifiProfileKey(index, "priority").c_str());
  }
  prefs.remove("net_count");
}

void sortSavedWifiProfiles(SavedWifiProfile profiles[], size_t count) {
  for (size_t i = 1; i < count; ++i) {
    SavedWifiProfile current = profiles[i];
    size_t j = i;
    while (j > 0 && profiles[j - 1].priority < current.priority) {
      profiles[j] = profiles[j - 1];
      --j;
    }
    profiles[j] = current;
  }
}

void loadSavedWifiProfiles(SavedWifiProfile profiles[], size_t& count) {
  count = 0;
  Preferences prefs;
  if (!prefs.begin(kWifiPrefsNamespace, true)) {
    return;
  }
  const size_t stored_count = static_cast<size_t>(prefs.getUChar("net_count", 0));
  const size_t max_count = stored_count < kMaxSavedWifiNetworks ? stored_count
                                                               : kMaxSavedWifiNetworks;
  for (size_t index = 0; index < max_count; ++index) {
    const String ssid = prefs.getString(wifiProfileKey(index, "ssid").c_str(), "");
    if (ssid.length() == 0) {
      continue;
    }
    SavedWifiProfile& profile = profiles[count++];
    profile.ssid = ssid;
    profile.pass = prefs.getString(wifiProfileKey(index, "pass").c_str(), "");
    profile.lat = prefs.getDouble(wifiProfileKey(index, "lat").c_str(),
                                 config::kDefaultRadarLat);
    profile.lon = prefs.getDouble(wifiProfileKey(index, "lon").c_str(),
                                 config::kDefaultRadarLon);
    profile.has_location = prefs.getBool(wifiProfileKey(index, "has_loc").c_str(), false);
    profile.priority = prefs.getUInt(wifiProfileKey(index, "priority").c_str(), 0);
  }
  prefs.end();
  sortSavedWifiProfiles(profiles, count);
}

void saveSavedWifiProfiles(const SavedWifiProfile profiles[], size_t count) {
  Preferences prefs;
  if (!prefs.begin(kWifiPrefsNamespace, false)) {
    return;
  }
  removeSavedWifiProfileKeys(prefs);
  prefs.putUChar("net_count", static_cast<uint8_t>(count));
  for (size_t index = 0; index < count; ++index) {
    const SavedWifiProfile& profile = profiles[index];
    prefs.putString(wifiProfileKey(index, "ssid").c_str(), profile.ssid);
    prefs.putString(wifiProfileKey(index, "pass").c_str(), profile.pass);
    prefs.putDouble(wifiProfileKey(index, "lat").c_str(), profile.lat);
    prefs.putDouble(wifiProfileKey(index, "lon").c_str(), profile.lon);
    prefs.putBool(wifiProfileKey(index, "has_loc").c_str(), profile.has_location);
    prefs.putUInt(wifiProfileKey(index, "priority").c_str(), profile.priority);
  }
  prefs.end();
}

void persistWifiProfile(const String& ssid, const String& pass, double lat,
                       double lon, bool has_location, uint32_t priority) {
  if (ssid.length() == 0) {
    return;
  }

  SavedWifiProfile profiles[kMaxSavedWifiNetworks];
  size_t count = 0;
  loadSavedWifiProfiles(profiles, count);

  size_t slot = count;
  for (size_t index = 0; index < count; ++index) {
    if (profiles[index].ssid.equalsIgnoreCase(ssid)) {
      slot = index;
      break;
    }
  }

  if (slot >= count) {
    if (count >= kMaxSavedWifiNetworks) {
      size_t lowest_index = 0;
      for (size_t index = 1; index < count; ++index) {
        if (profiles[index].priority < profiles[lowest_index].priority) {
          lowest_index = index;
        }
      }
      for (size_t index = lowest_index + 1; index < count; ++index) {
        profiles[index - 1] = profiles[index];
      }
      count = count > 0 ? count - 1 : 0;
      slot = count;
    }
    if (count < kMaxSavedWifiNetworks) {
      ++count;
      slot = count - 1;
    }
  }

  SavedWifiProfile& profile = profiles[slot];
  profile.ssid = ssid;
  profile.pass = pass;
  profile.lat = lat;
  profile.lon = lon;
  profile.has_location = has_location;

  if (priority != 0) {
    profile.priority = priority;
  } else {
    uint32_t highest_priority = 0;
    for (size_t index = 0; index < count; ++index) {
      if (profiles[index].priority > highest_priority) {
        highest_priority = profiles[index].priority;
      }
    }
    profile.priority = highest_priority + 1;
  }

  sortSavedWifiProfiles(profiles, count);
  saveSavedWifiProfiles(profiles, count);
}

bool applyNetworkLocationForCurrentSsid() {
  const String ssid = WiFi.SSID();
  if (ssid.length() == 0) {
    return false;
  }

  SavedWifiProfile profiles[kMaxSavedWifiNetworks];
  size_t count = 0;
  loadSavedWifiProfiles(profiles, count);
  for (size_t index = 0; index < count; ++index) {
    if (profiles[index].ssid.equalsIgnoreCase(ssid) && profiles[index].has_location) {
      services::location::set(profiles[index].lat, profiles[index].lon);
      return true;
    }
  }
  return false;
}

void markMostRecentWifiNetwork(const String& ssid) {
  if (ssid.length() == 0) {
    return;
  }

  SavedWifiProfile profiles[kMaxSavedWifiNetworks];
  size_t count = 0;
  loadSavedWifiProfiles(profiles, count);

  uint32_t highest_priority = 0;
  for (size_t index = 0; index < count; ++index) {
    if (profiles[index].priority > highest_priority) {
      highest_priority = profiles[index].priority;
    }
  }

  for (size_t index = 0; index < count; ++index) {
    if (profiles[index].ssid.equalsIgnoreCase(ssid)) {
      profiles[index].priority = highest_priority + 1;
      sortSavedWifiProfiles(profiles, count);
      saveSavedWifiProfiles(profiles, count);
      return;
    }
  }

  persistWifiProfile(ssid, s_wm.getWiFiPass(), services::location::lat(),
                    services::location::lon(), true, highest_priority + 1);
}

void saveCurrentWifiProfileLocation() {
  const String ssid = (WiFi.status() == WL_CONNECTED) ? WiFi.SSID() : s_wm.getWiFiSSID();
  const String pass = s_wm.getWiFiPass();
  if (ssid.length() == 0) {
    return;
  }
  persistWifiProfile(ssid, pass, services::location::lat(), services::location::lon(), true,
                    0);
}

constexpr int kCoordParamLen = 20;
constexpr char kCoordInputAttrs[] =
    " type=\"number\" step=\"0.000001\"";

WiFiManagerParameter s_param_lat("radar_lat", "Latitude (deg)", "0",
                                kCoordParamLen, kCoordInputAttrs);
WiFiManagerParameter s_param_lon("radar_lon", "Longitude (deg)", "0",
                                kCoordParamLen, kCoordInputAttrs);

char s_miles_checkbox_attrs[32] = "type=\"checkbox\"";
WiFiManagerParameter s_param_miles("use_miles", "Display distances in miles", "T", 2,
                                   s_miles_checkbox_attrs, WFM_LABEL_AFTER);

char s_runways_checkbox_attrs[32] = "type=\"checkbox\"";
WiFiManagerParameter s_param_runways("show_runways", "Show airport runways", "T", 2,
                                     s_runways_checkbox_attrs, WFM_LABEL_AFTER);

void refreshPortalParamDefaults() {
  char lat_buf[kCoordParamLen + 1];
  char lon_buf[kCoordParamLen + 1];
  snprintf(lat_buf, sizeof(lat_buf), "%.6f", services::location::lat());
  snprintf(lon_buf, sizeof(lon_buf), "%.6f", services::location::lon());
  s_param_lat.setValue(lat_buf, kCoordParamLen);
  s_param_lon.setValue(lon_buf, kCoordParamLen);
  snprintf(s_miles_checkbox_attrs, sizeof(s_miles_checkbox_attrs), "type=\"checkbox\"%s",
           ui::radar::useMiles() ? " checked" : "");
  s_param_miles.setValue("T", 2);
  snprintf(s_runways_checkbox_attrs, sizeof(s_runways_checkbox_attrs),
           "type=\"checkbox\"%s", ui::radar::showRunways() ? " checked" : "");
  s_param_runways.setValue("T", 2);
}

void onPortalParamsSaved() {
  double lat = 0.0;
  double lon = 0.0;
  if (!services::location::saveFromStrings(s_param_lat.getValue(),
                                           s_param_lon.getValue())) {
    Serial.println("Invalid lat/lon in portal — keeping previous location");
  } else {
    lat = services::location::lat();
    lon = services::location::lon();
    const String ssid = s_wm.getWiFiSSID();
    const String pass = s_wm.getWiFiPass();
    if (ssid.length() > 0) {
      persistWifiProfile(ssid, pass, lat, lon, true, 0);
    }
  }
  ui::radar::saveMilesFromPortal(s_param_miles.getValue());
  ui::radar::saveRunwaysFromPortal(s_param_runways.getValue());
}

void attachPortalParams(WiFiManager& wm) {
  refreshPortalParamDefaults();
  wm.addParameter(&s_param_lat);
  wm.addParameter(&s_param_lon);
  wm.addParameter(&s_param_miles);
  wm.addParameter(&s_param_runways);
  wm.setSaveParamsCallback(onPortalParamsSaved);
}

void markForceConfigPortal() {
  s_force_config_portal = true;
  Preferences prefs;
  if (!prefs.begin(kWifiPrefsNamespace, false)) {
    return;
  }
  prefs.putBool(kPrefsForcePortalKey, true);
  prefs.end();
}

bool consumeForceConfigPortal() {
  if (s_force_config_portal) {
    s_force_config_portal = false;
    Preferences prefs;
    if (prefs.begin(kWifiPrefsNamespace, false)) {
      prefs.remove(kPrefsForcePortalKey);
      prefs.end();
    }
    return true;
  }

  Preferences prefs;
  if (!prefs.begin(kWifiPrefsNamespace, true)) {
    return false;
  }
  const bool pending = prefs.getBool(kPrefsForcePortalKey, false);
  prefs.end();
  if (!pending) {
    return false;
  }

  if (prefs.begin(kWifiPrefsNamespace, false)) {
    prefs.remove(kPrefsForcePortalKey);
    prefs.end();
  }
  return true;
}

bool storedWifiCredentials() {
  SavedWifiProfile profiles[kMaxSavedWifiNetworks];
  size_t count = 0;
  loadSavedWifiProfiles(profiles, count);
  return count > 0;
}

void eraseWifiCredentials() {
  stopLanWebPortal();
  WiFi.setAutoReconnect(false);
  WiFi.mode(WIFI_OFF);
  delay(100);

  Preferences prefs;
  if (prefs.begin(kWifiPrefsNamespace, false)) {
    removeSavedWifiProfileKeys(prefs);
    prefs.end();
  }

  ensureWifiManager();
  WiFi.persistent(true);
  s_wm.resetSettings();
  s_wm.erase();
  WiFi.disconnect(true, true);
  WiFi.persistent(false);

  WiFi.mode(WIFI_OFF);
  delay(100);
}

void resetWifiCredentials() {
  markForceConfigPortal();
  eraseWifiCredentials();
  services::location::clear();
  ui::radar::unitsReset();
  Serial.println("WiFi credentials, location, and units cleared");
}

void onConfigPortalApStarted(WiFiManager*) {
  WiFi.setTxPower(WIFI_POWER_8_5dBm);
  statusScreenPortal();
#ifdef WM_MDNS
  if (MDNS.begin(config::kPortalHostname)) {
    MDNS.addService("http", "tcp", 80);
    Serial.printf("Setup portal: http://%s.local (or http://%s)\n",
                  config::kPortalHostname, config::kPortalIp);
  } else {
    Serial.printf("Setup portal: http://%s (mDNS unavailable)\n", config::kPortalIp);
  }
#else
  Serial.printf("Setup portal: http://%s\n", config::kPortalIp);
#endif
}

bool wifiLinkUp() {
  return WiFi.status() == WL_CONNECTED &&
         WiFi.localIP() != IPAddress(0, 0, 0, 0);
}

void ensureWifiManager() {
  if (s_wm_configured) {
    return;
  }
  s_wm.setConfigPortalTimeout(config::kWifiPortalTimeoutSec);
  s_wm.setAPStaticIPConfig(IPAddress(192, 168, 4, 1), IPAddress(192, 168, 4, 1),
                           IPAddress(255, 255, 255, 0));
  s_wm.setHostname(config::kPortalHostname);
  s_wm.setAPCallback(onConfigPortalApStarted);
  attachPortalParams(s_wm);
  s_wm_configured = true;
}

void startLanWebPortal() {
  if (!wifiLinkUp() || s_wm.getWebPortalActive() ||
      s_wm.getConfigPortalActive()) {
    return;
  }
  refreshPortalParamDefaults();
  WiFi.mode(WIFI_STA);
  s_wm.setConfigPortalBlocking(false);
#ifdef WM_MDNS
  MDNS.end();
  if (MDNS.begin(config::kPortalHostname)) {
    MDNS.addService("http", "tcp", 80);
  }
#endif
  s_wm.startWebPortal();
  Serial.printf("LAN config: http://%s.local or http://%s\n",
                config::kPortalHostname, WiFi.localIP().toString().c_str());
}

void stopLanWebPortal() {
  if (!s_wm.getWebPortalActive()) {
    return;
  }
  s_wm.stopWebPortal();
#ifdef WM_MDNS
  MDNS.end();
#endif
}

void prepareSta() {
  WiFi.setTxPower(WIFI_POWER_8_5dBm);
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(WIFI_PS_NONE);
  WiFi.setAutoReconnect(true);
}

void startStaConnect(const String& ssid, const String& pass) {
  prepareSta();
  if (ssid.length() > 0) {
    WiFi.begin(ssid.c_str(), pass.c_str());
  } else {
    WiFi.begin();
  }
}

bool waitForLinkWithUi(const char* ssid_for_ui, unsigned long attempt_ms) {
  const unsigned long deadline = millis() + attempt_ms;
  while (millis() < deadline) {
    if (wifiLinkUp()) {
      return true;
    }
    bootButtonPollLongPress();
    statusScreenConnectingTick();
    delay(config::kWifiConnectingFrameMs);
  }
  return wifiLinkUp();
}

bool tryConnectWithUi(const String& ssid, const String& pass, bool show_ui) {
  if (wifiLinkUp()) {
    return true;
  }

  const char* ui_ssid = ssid.length() > 0 ? ssid.c_str() : "network";
  if (show_ui) {
    statusScreenConnectingBegin(ui_ssid);
  }

  for (uint8_t attempt = 1; attempt <= config::kWifiConnectAttempts; ++attempt) {
    if (attempt > 1) {
      Serial.printf("WiFi connect retry %u/%u\n", attempt,
                    config::kWifiConnectAttempts);
      WiFi.disconnect(true);
      WiFi.mode(WIFI_OFF);
      delay(400);
    }

    startStaConnect(ssid, pass);

    if (waitForLinkWithUi(ui_ssid, config::kWifiConnectAttemptMs)) {
      return true;
    }
  }

  return false;
}

bool connectSavedNetwork(bool show_ui) {
  SavedWifiProfile profiles[kMaxSavedWifiNetworks];
  size_t count = 0;
  loadSavedWifiProfiles(profiles, count);
  if (count == 0) {
    return false;
  }

  ensureWifiManager();
  for (size_t index = 0; index < count; ++index) {
    const SavedWifiProfile& profile = profiles[index];
    if (profile.ssid.length() == 0) {
      continue;
    }
    if (tryConnectWithUi(profile.ssid, profile.pass, show_ui)) {
      if (applyNetworkLocationForCurrentSsid()) {
        Serial.printf("Applied saved location for %s\n", WiFi.SSID().c_str());
      }
      markMostRecentWifiNetwork(WiFi.SSID());
      return true;
    }
  }
  return false;
}

bool openConfigPortal() {
  stopLanWebPortal();
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  delay(50);
  statusScreenPortal();
  s_wm.setConfigPortalBlocking(false);
  s_wm.startConfigPortal(config::kPortalApName);
  while (s_wm.getConfigPortalActive()) {
    bootButtonPollLongPress();
    if (s_wm.process()) {
      return true;
    }
    delay(10);
  }
  return wifiLinkUp();
}

}  // namespace

bool wifiShowsSetupScreenOnBoot() {
  if (s_force_config_portal) {
    return true;
  }
  Preferences prefs;
  if (!prefs.begin(kWifiPrefsNamespace, true)) {
    return false;
  }
  const bool pending = prefs.getBool(kPrefsForcePortalKey, false);
  prefs.end();
  return pending;
}

bool wifiBootButtonPressed() {
  return digitalRead(config::kBootPin) == LOW;
}

void bootButtonInit() { initBootButton(); }

bool bootButtonConsumeTap() {
  portENTER_CRITICAL(&s_boot_mux);
  const bool tap = s_boot_tap_pending;
  if (tap) {
    s_boot_tap_pending = false;
  }
  portEXIT_CRITICAL(&s_boot_mux);
  return tap;
}

void bootButtonPollLongPress() {
  if (wifiBootButtonPressed()) {
    portENTER_CRITICAL(&s_boot_mux);
    if (!s_boot_is_down) {
      s_boot_is_down = true;
      s_boot_down_ms = millis();
    }
    const unsigned long down_ms = s_boot_down_ms;
    portEXIT_CRITICAL(&s_boot_mux);

    if (!s_long_press_handled &&
        millis() - down_ms >= config::kBootResetHoldMs) {
      s_long_press_handled = true;
      Serial.println("BOOT held — resetting WiFi");
      wifiResetCredentialsAndReboot();
    }
  } else {
    portENTER_CRITICAL(&s_boot_mux);
    s_boot_is_down = false;
    portEXIT_CRITICAL(&s_boot_mux);
    s_long_press_handled = false;
  }
}

void wifiResetCredentialsAndReboot() {
  resetWifiCredentials();
  statusScreenWifiReset();
  delay(800);
  esp_restart();
}

bool wifiReconnect() {
  initBootButton();
  Serial.println("WiFi reconnecting...");
  return connectSavedNetwork(true);
}

void wifiLoop() {
  ensureWifiManager();
  if (wifiLinkUp()) {
    if (!s_wm.getWebPortalActive() && !s_wm.getConfigPortalActive()) {
      startLanWebPortal();
    }
    if (s_wm.getWebPortalActive() || s_wm.getConfigPortalActive()) {
      bootButtonPollLongPress();
      s_wm.process();
    }
  } else {
    stopLanWebPortal();
  }
}

bool wifiSetupConnect() {
  initBootButton();
  ensureWifiManager();

  const bool force_portal = consumeForceConfigPortal();
  WiFi.setAutoReconnect(false);

  if (force_portal) {
    eraseWifiCredentials();
    WiFi.mode(WIFI_OFF);
    delay(100);
  }

  if (force_portal) {
    Serial.println("Opening WiFi setup portal (after reset)");
    if (openConfigPortal() && wifiLinkUp()) {
      WiFi.setAutoReconnect(true);
      Serial.printf("Connected: %s  IP %s\n", WiFi.SSID().c_str(),
                    WiFi.localIP().toString().c_str());
      return true;
    }
    Serial.println("WiFi connection failed");
    statusScreenConnectFailed();
    return false;
  }

  Serial.println("Connecting to WiFi (portal opens if needed)...");

  if (wifiLinkUp()) {
    WiFi.setAutoReconnect(true);
    Serial.printf("Connected: %s  IP %s\n", WiFi.SSID().c_str(),
                  WiFi.localIP().toString().c_str());
    return true;
  }

  if (storedWifiCredentials() && connectSavedNetwork(true)) {
    WiFi.setAutoReconnect(true);
    Serial.printf("Connected: %s  IP %s\n", WiFi.SSID().c_str(),
                  WiFi.localIP().toString().c_str());
    return true;
  }

  if (storedWifiCredentials()) {
    Serial.println("Saved WiFi could not connect — opening setup portal");
  } else {
    Serial.println("No saved WiFi — opening setup portal");
  }

  if (openConfigPortal() && wifiLinkUp()) {
    WiFi.setAutoReconnect(true);
    Serial.printf("Connected: %s  IP %s\n", WiFi.SSID().c_str(),
                  WiFi.localIP().toString().c_str());
    return true;
  }

  Serial.println("WiFi connection failed");
  statusScreenConnectFailed();
  return false;
}
