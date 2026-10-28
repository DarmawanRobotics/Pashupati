# pashupati_utils

Helpers the other Pashupati packages share. No node.

| Module                    |                                                                                                                                                                         |
| ------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `core/numbers.py`         | finite numbers from the network and for JSON                                                                                                                            |
| `core/system.py`          | the computer's figures (CPU, GPU, RAM, temperature, disk, load) and the sensor rates from the findings `health_monitor_node` publishes on `diagnostics`                 |
| `utils/status.py`         | `ControlStatus`, `NavigationStatus`, `LocalizationStatus`, `StopPointEvent`, `HealthStatus` as the values of the handheld protocol (`PROTOCOL.md`) and the NETRA API |
| `utils/params.py`         | `SetParameters` requests for another node, `(ok, message)` of any service answer                                                                                        |
| `utils/calls.py`          | an asynchronous service call that reports `(ok, message)`                                                                                                               |

```bash
colcon test --packages-select pashupati_utils && colcon test-result --verbose
```
