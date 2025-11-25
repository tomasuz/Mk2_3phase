#!/usr/bin/env python3
# TinyTuya → MQTT bridge tailored for your Tuya EV Charger
# - Publishes per-DPS state under:  tuya/<devId>/dps/<dp>
# - Accepts per-DPS commands at:    tuya/<devId>/dps/<dp>/set
# - Publishes full DPS JSON at:     tuya/<devId>/state
# - Availability at:                tuya/<devId>/availability
#
# Home Assistant MQTT Discovery is enabled and uses the DPS map
# (units/scale/ranges) from the device JSON you provided.

import os, json, time, threading, queue
from typing import Any, Dict
import paho.mqtt.client as mqtt
import tinytuya

# ------------------------- Config via env -------------------------
MQTT_HOST   = os.getenv("MQTT_HOST", "127.0.0.1")
MQTT_PORT   = int(os.getenv("MQTT_PORT", "1883"))
MQTT_USER   = os.getenv("MQTT_USER", "")
MQTT_PASS   = os.getenv("MQTT_PASS", "")
MQTT_BASE   = os.getenv("MQTT_BASE", "tuya")
POLL_SEC    = float(os.getenv("POLL_SEC", "2.5"))
TIMEOUT     = float(os.getenv("TUYA_TIMEOUT", "5.0"))
RETRIES     = int(os.getenv("TUYA_RETRIES", "3"))
DEVICES_JSON= os.getenv("DEVICES_JSON", "/mnt/data/device1.json")  # your uploaded file
DISC_PREFIX = os.getenv("DISC_PREFIX", "homeassistant")            # HA discovery prefix
DISCOVERY   = True  # keep True (we tailor discovery to your DPS)

# ------------------------- Helpers -------------------------
def mk_client() -> mqtt.Client:
    c = mqtt.Client()
    if MQTT_USER:
        c.username_pw_set(MQTT_USER, MQTT_PASS)
    c.will_set(f"{MQTT_BASE}/bridge/availability", "offline", retain=True)
    return c

def pub(client, topic, payload, retain=False):
    client.publish(topic, payload, qos=0, retain=retain)

def load_devices(path: str):
    with open(path, "r", encoding="utf-8") as f:
        data = json.load(f)
    # Accept either array or {"devices":[...]}
    if isinstance(data, dict) and "devices" in data:
        data = data["devices"]
    return data

def mk_tuya(dev_entry: Dict[str, Any]) -> tinytuya.Device:
    d = tinytuya.Device(dev_entry["id"], dev_entry.get("ip"), dev_entry["key"])
    ver = float(str(dev_entry.get("version", "3.3")))
    d.set_version(ver)
    d.set_socketPersistent(True)
    d.set_socketRetryLimit(RETRIES)
    d.set_socketTimeout(TIMEOUT)
    return d

# Scaling helpers using mapping from your JSON
def scale_out(dps_id: str, val: Any, mapping: Dict[str, Any]):
    """Scale device->MQTT (read path)"""
    m = mapping.get(dps_id)
    if not m:  # no mapping info
        return val
    t = m.get("type")
    if t == "Integer":
        scale = int(m.get("values", {}).get("scale", 0))
        try:
            return float(val) / (10 ** scale)
        except Exception:
            return val
    return val

def scale_in(dps_id: str, val: Any, mapping: Dict[str, Any]):
    """Scale MQTT->device (write path)"""
    m = mapping.get(dps_id)
    if not m:
        return val
    t = m.get("type")
    if t == "Integer":
        scale = int(m.get("values", {}).get("scale", 0))
        try:
            # allow string/bool/number inputs
            if isinstance(val, str):
                if val.lower() in ("true", "false"):
                    val = (val.lower() == "true")
                else:
                    val = float(val)
            if isinstance(val, bool):
                return val
            return int(round(float(val) * (10 ** scale)))
        except Exception:
            return val
    elif t == "Boolean":
        if isinstance(val, str):
            return val.lower() == "true"
        return bool(val)
    elif t == "Enum":
        # expect exact enum string
        return str(val)
    return val

def ha_device_block(dev: Dict[str, Any]):
    name = dev.get("product_name") or dev.get("name") or f"Tuya {dev['id'][:6]}"
    mdl  = dev.get("model") or dev.get("category") or "Tuya Device"
    mf   = "Tuya"
    return {
        "identifiers": [dev["id"]],
        "manufacturer": mf,
        "model": mdl,
        "name": name,
        "sw_version": dev.get("version"),
    }

def publish_discovery(client, dev: Dict[str, Any]):
    """Create HA discovery entities based on your DPS map"""
    dev_id = dev["id"]
    mp = dev.get("mapping", {})
    device = ha_device_block(dev)

    def disc(domain, uniq, cfg):
        base = f"{DISC_PREFIX}/{domain}/{uniq}/config"
        cfg.setdefault("availability_topic", f"{MQTT_BASE}/{dev_id}/availability")
        cfg.setdefault("payload_available", "online")
        cfg.setdefault("payload_not_available", "offline")
        cfg["device"] = device
        pub(client, base, json.dumps(cfg), retain=True)

    # --- Switch (DPS 18: on/off) ---
    if "18" in mp and mp["18"]["type"] == "Boolean":
        uniq = f"{dev_id}_18_switch"
        disc("switch", uniq, {
            "name": "EV Charger - Switch",
            "unique_id": uniq,
            "state_topic":  f"{MQTT_BASE}/{dev_id}/dps/18",
            "command_topic":f"{MQTT_BASE}/{dev_id}/dps/18/set",
            "payload_on": "true",
            "payload_off":"false",
            "state_on": "true",
            "state_off":"false",
            "icon": "mdi:ev-station"
        })

    if "19" in mp and mp["19"]["type"] == "Integer":
        uniq = f"{dev_id}_19_current"
        disc("sensor", uniq, {
            "name": "EV Charger - Current",
            "unique_id": uniq,
            "state_topic": f"{MQTT_BASE}/{dev_id}/dps/19",
            "unit_of_measurement": "A",
            "device_class": "current",
            "icon": "mdi:current"
        })

    # --- Charge current set (DPS 4: Integer A) ---
    if "4" in mp and mp["4"]["type"] == "Integer":
        vals = mp["4"]["values"]; scale = int(vals.get("scale", 0))
        # We expose the *scaled* human value (A)
        uniq = f"{dev_id}_4_current"
        disc("number", uniq, {
            "name": "EV Charger - Max Current",
            "unique_id": uniq,
            "state_topic":   f"{MQTT_BASE}/{dev_id}/dps/4",
            "command_topic": f"{MQTT_BASE}/{dev_id}/dps/4/set",
            "unit_of_measurement": "A",
            "min": float(vals.get("min", 1)) / (10 ** scale),
            "max": float(vals.get("max", 16)) / (10 ** scale),
            "step": max(float(vals.get("step", 1)) / (10 ** scale), 0.1),
            "icon": "mdi:current-ac"
        })

    # --- Work mode (DPS 14: Enum) as select ---
    if "14" in mp and mp["14"]["type"] == "Enum":
        uniq = f"{dev_id}_14_mode"
        disc("select", uniq, {
            "name": "EV Charger - Work Mode",
            "unique_id": uniq,
            "state_topic":   f"{MQTT_BASE}/{dev_id}/dps/14",
            "command_topic": f"{MQTT_BASE}/{dev_id}/dps/14/set",
            "options": mp["14"]["values"]["range"],
            "icon": "mdi:calendar-clock"
        })

    # --- Work state (DPS 3: Enum) as sensor (read-only) ---
    if "3" in mp and mp["3"]["type"] == "Enum":
        uniq = f"{dev_id}_3_state"
        disc("sensor", uniq, {
            "name": "EV Charger - Work State",
            "unique_id": uniq,
            "state_topic": f"{MQTT_BASE}/{dev_id}/dps/3",
            "icon": "mdi:state-machine"
        })

    # --- Power (DPS 5 single-phase kW, DPS 9 total kW) ---
    for dps, nm in (("5","Single-phase Power"), ("9","Total Power")):
        if dps in mp and mp[dps]["type"] == "Integer":
            uniq = f"{dev_id}_{dps}_power"
            disc("sensor", uniq, {
                "name": f"EV Charger - {nm}",
                "unique_id": uniq,
                "state_topic": f"{MQTT_BASE}/{dev_id}/dps/{dps}",
                "unit_of_measurement": "kW",
                "device_class": "power",
                "icon": "mdi:flash"
            })

    # --- Energy totals (DPS 1 forward kWh, DPS 25 session kWh) ---
    for dps, nm in (("1","Energy Total"), ("25","Session Energy")):
        if dps in mp and mp[dps]["type"] == "Integer":
            uniq = f"{dev_id}_{dps}_energy"
            disc("sensor", uniq, {
                "name": f"EV Charger - {nm}",
                "unique_id": uniq,
                "state_topic": f"{MQTT_BASE}/{dev_id}/dps/{dps}",
                "unit_of_measurement": "kWh",
                "device_class": "energy",
                "state_class": "total_increasing" if dps=="1" else "measurement",
                "icon": "mdi:lightning-bolt-circle"
            })

    # --- Temperature (DPS 24 °C) ---
    if "24" in mp and mp["24"]["type"] == "Integer":
        uniq = f"{dev_id}_24_temp"
        disc("sensor", uniq, {
            "name": "EV Charger - Temperature",
            "unique_id": uniq,
            "state_topic": f"{MQTT_BASE}/{dev_id}/dps/24",
            "unit_of_measurement": "°C",
            "device_class": "temperature",
            "icon": "mdi:thermometer"
        })

def worker(dev_entry: Dict[str, Any], client: mqtt.Client):
    dev_id = dev_entry["id"]
    mapping = dev_entry.get("mapping", {})
    topic_base = f"{MQTT_BASE}/{dev_id}"

    # Device & connection
    d = mk_tuya(dev_entry)

    # LWT online
    pub(client, f"{topic_base}/availability", "online", retain=True)

    # Command handling (per-DP set)
    def on_msg(_c, _u, msg):
        try:
            # Expect topic: tuya/<id>/dps/<dp>/set
            parts = msg.topic.split("/")
            if len(parts) >= 5 and parts[-2].isdigit() and parts[-1] == "set":
                dp = parts[-2]
                raw = msg.payload.decode().strip()
                # try JSON -> fallbacks -> raw
                try:
                    val = json.loads(raw)
                except json.JSONDecodeError:
                    if raw.lower() in ("true","false"):
                        val = (raw.lower() == "true")
                    else:
                        try:
                            # number or pass-through string
                            val = float(raw) if "." in raw or raw.isdigit() else raw
                        except Exception:
                            val = raw
                val_scaled = scale_in(dp, val, mapping)
                try:
                    d.set_status(val_scaled, dp)
                except Exception as e:
                    print(f"[{dev_id}] set DPS {dp} failed: {e}")
        except Exception as e:
            print(f"[{dev_id}] command parse error: {e}")

    client.message_callback_add(f"{topic_base}/dps/+/set", on_msg)
    client.subscribe(f"{topic_base}/dps/+/set")

    last = {}
    while True:
        # poll
        try:
            status = d.status()  # {'dps': {...}}
            dps = status.get("dps", {})
            # publish per-DP (with scaling)
            changed = False
            for k, v in dps.items():
                scaled = scale_out(k, v, mapping)
                # Publish booleans as strings to match HA discovery expectations
                if last.get(k) != scaled:
                    if isinstance(scaled, bool):
                        # Publish as plain string "true" or "false" for HA switches
                        payload = "true" if scaled else "false"
                    else:
                        # Publish as JSON for numbers/strings/enums
                        payload = json.dumps(scaled)
                    pub(client, f"{topic_base}/dps/{k}", payload, retain=True)
                    last[k] = scaled
                    changed = True
            if changed:
                pub(client, f"{topic_base}/state", json.dumps(last), retain=True)
            pub(client, f"{topic_base}/availability", "online", retain=True)
        except Exception as e:
            # offline or timeout
            pub(client, f"{topic_base}/availability", "offline", retain=True)
            time.sleep(1.0)
        time.sleep(POLL_SEC)

def main():
    devs = load_devices(DEVICES_JSON)
    if not devs:
        raise SystemExit("No devices found in devices.json")

    # MQTT connect
    client = mk_client()
    client.connect(MQTT_HOST, MQTT_PORT, 60)
    client.loop_start()
    pub(client, f"{MQTT_BASE}/bridge/availability", "online", retain=True)

    # Discovery for each device
    if DISCOVERY:
        for dev in devs:
            publish_discovery(client, dev)

    # Start workers
    for dev in devs:
        t = threading.Thread(target=worker, args=(dev, client), daemon=True)
        t.start()

    # keep alive
    try:
        while True:
            time.sleep(3600)
    except KeyboardInterrupt:
        pass

if __name__ == "__main__":
    main()
