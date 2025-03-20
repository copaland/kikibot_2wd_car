#ifndef DEVICE_NAME_H
#define DEVICE_NAME_H

#include <Arduino.h>
#include <Preferences.h>  // ESP32 NVS 라이브러리

#define BT_NAME_PREFIX "KiKiBot_"

static const int MAX_COUNT = 999;

// 블루투스 이름을 생성하는 함수
String getBluetoothName() {
    Preferences preferences;
    preferences.begin("bt_config", false);  // "bt_config" 네임스페이스 사용

    // 저장된 블루투스 이름 가져오기
    String btName = preferences.getString("bt_name", "");

    // 만약 저장된 이름이 없으면 새 이름 생성 (최초 실행 시)
    if (btName == "") {
      // 저장된 블루투스 번호 가져오기 (기본값: 100)
      int btNumber = preferences.getInt("bt_number", 100);
      btNumber = (btNumber + 1) % MAX_COUNT;  // 번호 증가

      // 새로운 블루투스 이름 생성
      char newBtName[12];
      snprintf(newBtName, sizeof(newBtName), "%s%03d", BT_NAME_PREFIX, btNumber);
      btName = String(newBtName);

      // 새로운 블루투스 이름 및 번호 저장
      preferences.putString("bt_name", btName);
      preferences.putInt("bt_number", btNumber);
    }
    preferences.end();

    return btName;
}

void resetBluetoothCounter() {
    Preferences preferences;
    preferences.begin("bt_config", false);  // NVS 네임스페이스 열기

    // 저장된 값 삭제
    preferences.remove("bt_number");
    preferences.remove("bt_name");

    Serial.println("블루투스 카운트 초기화 완료! (다음 실행 시 01부터 시작)");
    
    preferences.end();
}
#endif // DEVICE_NAME_H
