// Заглушка Adafruit_NeoPixel для хост-симуляции: цвет пикселя хранится как
// 0x00RRGGBB, яркость применяется на show() — как в настоящей библиотеке.
// Результат show() кладётся в sim_board.pixels, поэтому сценарий может
// проверить, что реально ушло в ленту.
// Include guard намеренно нет: у каждой платы своя лента.
#include "../arduino_sim.h"

#define NEO_RGB 0x06
#define NEO_GRB 0x52
#define NEO_KHZ800 0x0000
#define NEO_KHZ400 0x0100

class Adafruit_NeoPixel {
 public:
  Adafruit_NeoPixel(uint16_t count = 1, uint8_t pin = 6, uint16_t type = NEO_GRB + NEO_KHZ800)
      : count_(count), pin_(pin), type_(type) {}

  void begin() {
    buffer_.assign(count_, 0);
    sim_board.pixels.assign(count_, 0);
  }
  void setBrightness(uint8_t brightness) { brightness_ = brightness; }
  uint8_t getBrightness() const { return brightness_; }
  uint16_t numPixels() const { return count_; }
  uint8_t getPin() const { return pin_; }
  uint16_t getType() const { return type_; }

  void clear() { std::fill(buffer_.begin(), buffer_.end(), 0u); }

  void setPixelColor(uint16_t index, uint32_t color) {
    if (index < buffer_.size()) buffer_[index] = color;
  }
  uint32_t getPixelColor(uint16_t index) const {
    return index < buffer_.size() ? buffer_[index] : 0;
  }

  void show() {
    sim_board.pixels.resize(count_);
    for (uint16_t i = 0; i < count_; ++i) {
      const uint32_t color = i < buffer_.size() ? buffer_[i] : 0;
      const uint8_t red = (uint8_t)((color >> 16) & 0xFF);
      const uint8_t green = (uint8_t)((color >> 8) & 0xFF);
      const uint8_t blue = (uint8_t)(color & 0xFF);
      const uint8_t scale = brightness_;
      sim_board.pixels[i] = ((uint32_t)((uint16_t)red * scale / 255) << 16) |
                            ((uint32_t)((uint16_t)green * scale / 255) << 8) |
                            (uint32_t)((uint16_t)blue * scale / 255);
    }
    sim_board.strip_brightness = brightness_;
    sim_board.strip_shows += 1;
  }

  static uint32_t Color(uint8_t red, uint8_t green, uint8_t blue) {
    return ((uint32_t)red << 16) | ((uint32_t)green << 8) | (uint32_t)blue;
  }

 private:
  uint16_t count_ = 1;
  uint8_t pin_ = 6;
  uint16_t type_ = 0;
  uint8_t brightness_ = 255;
  std::vector<uint32_t> buffer_;
};
