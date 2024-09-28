const int ledPin = 7; // LED가 연결된 핀 번호

void setup() {
    pinMode(ledPin, OUTPUT); // LED 핀을 출력으로 설정
}

void loop() {
    // 1초 동안 LED 끄기
    digitalWrite(ledPin, LOW); // LED 꺼짐
    delay(1000); // 1초 대기

    // LED를 1초 동안 5번 깜빡이기
    for (int i = 0; i < 5; i++) {
        digitalWrite(ledPin, HIGH); // LED 켜기
        delay(100); // 0.1초 대기
        digitalWrite(ledPin, LOW);  // LED 끄기
        delay(100); // 0.1초 대기
    }

    // LED를 켜고 무한 루프 상태로 종료
    digitalWrite(ledPin, HIGH); // LED 켜기
    while (true) {
        // LED가 켜진 상태로 유지
    }
}
