PY := mcp_server/.venv/bin/python
export ELLAMA_SERIAL_PORT := /dev/none

.PHONY: test check
test: ## Run the hardware-free test suites (script-style runners in mcp_server/)
	$(PY) mcp_server/test_sensors.py
	$(PY) mcp_server/test_tilt_guard.py
	$(PY) mcp_server/test_drive_feedback.py

check: ## Byte-compile all Python sources
	python3 -m py_compile bridge/__init__.py calibration/*.py mcp_server/*.py tools/*.py
