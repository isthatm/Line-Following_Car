- This repo contains source code of a three-wheel line-following car implemented on STM32 board.
- The speed of the car is adjusted dynamically based on the information regarding line position collected
  from four light-detecting sensors mounted at the bottom of the car.
- The information regarding speed, distance during the operation is transmitted to a local PC (serving as MQTT client)
  using MQTT broker.
