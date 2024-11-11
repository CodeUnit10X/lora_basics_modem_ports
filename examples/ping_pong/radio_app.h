#if !defined(_RADIO_APP_H_)
#define _RADIO_APP_H_

#include <cstdint>

class RadioApp {
public:
	RadioApp(uint32_t freq) : freq_{freq} {}

	~RadioApp() {}

	virtual void Run() = 0;
	virtual void TransmitBuffer(const uint8_t* buffer, const uint8_t size) = 0;
	virtual void InitRadio() = 0;
	virtual void ProcessIrq() = 0;

protected:
	uint32_t freq_;
};

#endif