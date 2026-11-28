PY := mcp_server/.venv/bin/python
export ELLAMA_SERIAL_PORT := /dev/none

.PHONY: test check
test: ## Run the hardware-free test suites (script-style runners)
	$(PY) mcp_server/test_bridge_v2.py
	$(PY) mcp_server/test_two_board_enc.py
	$(PY) mcp_server/test_robot_profiles.py
	$(PY) mcp_server/test_trace.py
	$(PY) mcp_server/test_dashboard_drive.py
	$(PY) mcp_server/test_sensors.py
	$(PY) mcp_server/test_tilt_guard.py
	$(PY) mcp_server/test_drive_feedback.py
	$(PY) mcp_server/test_operator_stop.py
	$(PY) mcp_server/test_drive_lease.py
	$(PY) mcp_server/test_vetbook.py
	$(PY) mcp_server/test_yaw_zupt.py
	$(PY) mcp_server/test_apriltag_module.py
	$(PY) mcp_server/test_tag_turn.py
	$(PY) mcp_server/test_plan_module.py
	$(PY) tools/test_realtime_map.py

check: ## Byte-compile all Python sources
	python3 -m py_compile bridge/__init__.py calibration/*.py mcp_server/*.py tools/*.py
