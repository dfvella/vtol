BOARD = arduino:avr:nano
PORT = /dev/ttyUSB0

flight_controller:
	arduino --board $(BOARD) --port $(PORT) --upload $@/main.ino

imu_calibration:
	arduino --board $(BOARD) --port $(PORT) --upload $@/$@.ino

servo_test:
	arduino --board $(BOARD) --port $(PORT) --upload $@/$@.ino

.PHONY: flight_controller imu_calibration servo_test
