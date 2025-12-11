# **E-Stop Manager**

The **E-Stop Manager** unifies multiple emergency-stop (E-stop) sources into consistent, aggregated E-stop states.
It supports two types of E-stop sources:

### **1. Tracked E-Stop Sources**

* The manager **subscribes** to a topic published by the source.
* The E-stop state is fully determined by the incoming `std_msgs/Bool` message.
* Topic name:

  ```
  ~/<e_stop_name>
  ```

### **2. Managed E-Stop Sources**

* These do **not** publish a topic themselves.
* The manager **hosts** the topic (publishes the current value).
* External components can update the state via the service:

  ```
  /e_stop_manager/set_e_stop
  ```

Both types of sources are combined into aggregated E-stop groups (e.g. *hardware*, *software*) and into a single combined E-stop list message.

---

# **Published Topics**

### **1. Aggregated E-Stop States (per group)**

Each aggregated group is published as a `std_msgs/Bool`:

```
<node_name>/aggregated_state/<aggregated_e_stop_name>    # e.g. /e_stop_manager/aggregated_state/emergency_stop_hardware
```

### **2. Complete E-Stop Status (all sources + all aggregates)**

```
<node_name>/e_stop_list   (e_stop_manager_msgs/EStopList)
```

The message contains:

* `names[]` — names of all individual E-stop sources
* `values[]` — their current states
* `aggregated_names[]` — names of all aggregated groups
* `aggregated_values[]` — current aggregated states
* All publishers and subscribers (tracked, managed, aggregated, list) use reliable + transient local QoS for latched, reliable delivery.
* Aggregated topic names are sanitized to valid ROS topic fragments (slashes removed, `-` becomes `_`). Invalid e-stop source names are rejected at startup.

---

# **Service API: Setting Managed E-Stops**

Managed E-stop sources can be updated via:

```
/e_stop_manager/set_e_stop
```

### **Request**

| Field   | Type   | Description                            |
| ------- | ------ | -------------------------------------- |
| `name`  | string | E-stop name (must exist in the config) |
| `value` | bool   | `true` = activated, `false` = released |

### **Response**

| Field    | Type | Description      |
| -------- | ---- | ---------------- |
| `result` | int8 | Operation result |

### **Result Constants**

| Name                 | Value | Meaning             |
| -------------------- | ----- | ------------------- |
| `SUCCESS`            | 0     | Update accepted     |
| `FAILURE`            | 1     | Update rejected     |
| `INVALID_ESTOP_NAME` | 2     | Unknown E-stop name |

---

# **Configuration**

All configuration is provided via ROS parameters.

### **Required parameters**

#### **`e_stop_names`**

List of all E-stop source names.

#### **`e_stop_config` (per source)**

Each source must specify:

| Field              | Type   | Meaning                                                  |
| ------------------ | ------ | -------------------------------------------------------- |
| `aggregated_topic` | string | The group this source belongs to                         |
| `tracked_topic`    | bool   | `true`: subscribe to `~/<name>`; `false`: managed source |
| `initial_value`    | bool   | Startup state                                            |

All aggregated groups are derived from the `aggregated_topic` values in `e_stop_config`, so no separate list is needed.

---

# **Example Configuration**

```yaml
e_stop_manager:
  ros__parameters:
    e_stop_names: ["hard_remote_e_stop", "soft_remote_e_stop", "big_red_button_e_stop", "ui_e_stop"]

    e_stop_config:
      hard_remote_e_stop:
        aggregated_topic: "/emergency_stop_hardware"
        tracked_topic: true          # subscribes to ~/<name>
        initial_value: false

      soft_remote_e_stop:
        aggregated_topic: "/emergency_stop_hardware"
        tracked_topic: true
        initial_value: false

      big_red_button_e_stop:
        aggregated_topic: "/emergency_stop_software"
        tracked_topic: true
        initial_value: false

      ui_e_stop:
        aggregated_topic: "/emergency_stop_software"
        tracked_topic: false         # managed source
        initial_value: false
```
