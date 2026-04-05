#pragma once
#include <Arduino.h>
#include <vector>

// 型を判別するためのenum
enum DataType {
    TYPE_INT,
    TYPE_FLOAT
};

// 変数の情報をまとめた構造体
struct Param {
    String name;
    void* addr;    // どんな型のポインタでも受け入れられる魔法のポインタ
    DataType type; // 元が何型だったかを覚えておく
};

class SerialManager {
private:
    std::vector<Param> params;

public:
    // int型の変数を登録するメソッド
    void add(const String& name, int* addr) {
        params.push_back({name, (void*)addr, TYPE_INT});
    }

    // float型の変数を登録するメソッド（オーバーロード）
    void add(const String& name, float* addr) {
        params.push_back({name, (void*)addr, TYPE_FLOAT});
    }

    // 初期化情報を全送信
    void sendAllInfo() {
        for (auto& p : params) {
            Serial.print("INIT:");
            Serial.print(p.name);
            Serial.print(":");
            
            // enumを見て元の型にキャスト（変換）して中身を取り出す
            if (p.type == TYPE_INT) {
                Serial.println(*(int*)p.addr);
            } else if (p.type == TYPE_FLOAT) {
                Serial.println(*(float*)p.addr);
            }
        }
    }

    // データの受信と変数の書き換え
    void update() {
        while (Serial.available() > 0) {
            String data = Serial.readStringUntil('\n');
            data.trim();

            int splitIndex = data.indexOf(':');
            if (splitIndex > 0) {
                String reqName = data.substring(0, splitIndex);
                String reqValueStr = data.substring(splitIndex + 1);

                for (auto& p : params) {
                    if (p.name == reqName) {
                        // enumを見て、適切な型に変換してポインタ先に代入
                        if (p.type == TYPE_INT) {
                            *(int*)p.addr = reqValueStr.toInt();
                        } else if (p.type == TYPE_FLOAT) {
                            *(float*)p.addr = reqValueStr.toFloat();
                        }
                        break;
                    }
                }
            }
        }
    }
};