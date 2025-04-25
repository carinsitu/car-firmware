#ifndef STATUSCAST_HPP
#define STATUSCAST_HPP

#include <WiFiUdp.h>
#include <CarCtrl.hpp>
#include <queue>

class StatusCast {
	private:
		typedef enum : uint8_t {
			STATUS_NO_REPORT,
			STATUS_RSSI,
			STATUS_IR,
			STATUS_SIMULATION,
			STATUS_HEADLIGHTS,
			STATUS_COLOR,
			STATUS_BATTERY,
			STATUS_IMU,
			STATUS_PILOT,
			STATUS_VERSION,
		} status_t;

		CarCtrl & _car;
		WiFiUDP _udp;
		unsigned long _last_msg = 0;
		uint8_t _last_msg_id = 0;
		
		// Queue for on-demand status reports
		std::queue<status_t> _on_demand_queue;
		
		// Helper method to add a status to the message buffer
		int addStatus(status_t status, uint8_t* msg, int len, int max_len);

	public:
		StatusCast(CarCtrl & car) : _car(car) {}
		void init();
		void loop();
		void send();
		
		// Request a specific status to be sent with the next broadcast
		void requestStatus(status_t status);
		
		// Helper method to specifically request version
		void requestVersion() { requestStatus(STATUS_VERSION); }
};

#endif
