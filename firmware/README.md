# firmware

STM32F446RE 펌웨어 (FreeRTOS + CMake).

## 빌드 도메인
- 툴체인: `arm-none-eabi-gcc`
- 빌드 도구: CMake
- 실시간 OS: FreeRTOS

## 예정 구조
- `Core/` — CubeMX 생성물
- `Drivers/` — HAL, CMSIS
- `Middlewares/` — FreeRTOS
- `App/` — 팀 작업 영역 (hal, drivers, control, gait, comm, tasks)
- `proto_generated/` — shared/proto에서 자동 생성 (gitignore)
- `Tests/` — Unity/CMock 단위 테스트

담당: HW