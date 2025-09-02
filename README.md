# Mycobot280

## 프로젝트 소개

본 프로젝트는 **Ubuntu 환경에서 직접 시리얼 통신을 구현**하여, **Mycobot280 로봇 암을 Raw Serial Packet 단위로 제어**하는 데 목적이 있습니다.
Elephant Robotics의 공식 SDK 는 ARM64 CPU를 지원하지 않아, Raspberry Pi 환경에서 C++로 로봇을 제어하기 위해 개발했습니다.
Elephant Robotics의 공식 SDK 대신, `/dev/ttyUSB*` (또는 Raspberry Pi 환경의 `/dev/ttyTHS1` 등)를 통해 직접 통신하며 제어합니다.

- 헤더 파일(`openRobotArm.hpp`)에는 함수 인터페이스가 정의되어 있고, 구현 파일(`openRobotArm.cpp`)에는 실제 통신 로직이 담겨 있습니다.
- **데이터 송수신 흐름**: 포트 열기 → 데이터 쓰기 → 응답 읽기 → 처리 → 동작 실행
- SDK 의존성을 최소화하여, **가볍고 확장 가능하며 명확한 구조**를 지향합니다.

---

## 프로젝트 소개

Mycobot280
├── include/
│ └── openRobotArm.hpp # 로봇 암 제어용 헤더 파일
└── src/
└── openRobotArm.cpp # 시리얼 통신 기반 로봇 암 제어 구현부

## 통신 프로토콜 (Elephant Robotics)
### Elephant Robotics
https://docs.elephantrobotics.com/docs/gitbook-en/18-communication/18-communication.html
