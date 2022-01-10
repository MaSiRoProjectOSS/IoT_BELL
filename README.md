# IoT_BELL

カフェの入店管理するシステムです。

* 入店時に予約番号がわかるもの（QRコードなど）を提示してもらうことで、来店を検知する。
* 卓におかれたIoTベルを鳴らすことで、スタッフに呼び出しがあったことを通知する。


## 全体構成

```plantuml
@startuml

rectangle outside_the_store as "店外" {
    actor actor_enter as "入店したいお客"
    frame tablet_side as "Tablet" {
        node Panel as "タッチパネル"
        node Raspberry as "タブレット\nRaspberry Pi"
    }
    node QRCodeReader as "QRコード\nリーダー"

}
rectangle inside_the_store as "店内" {
    person cast as "キャスト\nスタッフ"

    actor actor_order as "注文したいお客"
    frame bell_side as "IoT ベル" {
        node TWELITE_CUE as "TWELITE® CUE"
    }

}
rectangle backyard as "バックヤード" {
    frame manager as "サーバー" {
        node TWELITE_MONOSTICK as "MONOSTICK"
        node server as "サーバー\nfor Raspberry Pi 4"
    }
}

cloud cloud as "クラウド"


actor_order --> TWELITE_CUE
TWELITE_CUE --> TWELITE_MONOSTICK

actor_enter <-- Panel : 状態の通知
actor_enter --> QRCodeReader : QRコードの提示
QRCodeReader --> Raspberry : 読み取り\nデータ

Panel <-- Raspberry
TWELITE_MONOSTICK --> server : "TWELITE® CUE\nのデータ"

cast <--> server


Raspberry <--> server
server <--> cloud

@enduml

```

## 構成要素

### 利用しているハードウェア

| ハードウェア名       | 用途                                 | URL                                                          |
| -------------------- | ------------------------------------ | ------------------------------------------------------------ |
| Raspberry Pi 4       | 携帯サーバー 兼 タブレットの画面表示 | https://www.raspberrypi.com/products/raspberry-pi-4-model-b/ |
| TWELITE® CUE         | ベルのシェイク検知                   | https://mono-wireless.com/jp/products/twelite-cue/index.html |
| MONOSTICK            | TWELITE® CUEのシェイク検知を受信する | https://mono-wireless.com/jp/products/MoNoStick/index.html   |
| Raspberry Pi用カメラ | QRコードの読み取り                   | --                                                           |
| 店内用Wifi ルーター  | 各モジュール間で連絡するためのルータ | --                                                           |


### 利用しているソフトウェア

| ソフトウェア               | 用途                                                   | URL                                     |
| -------------------------- | ------------------------------------------------------ | --------------------------------------- |
| ROS 2 : Foxy               | ロボットの連携させるためのソフトウェアプラットフォーム | https://docs.ros.org/en/foxy/index.html |
| Ubuntu server 20.0.4.3 LTS | Raspberry Pi で動作させるOS                            | https://ubuntu.com/                     |


## 特記事項

本システムは**メイドロボカフェ**を運用するためにROSをプラットフォームにして構築している。


## License

* QRコードは(株)デンソーウェーブの登録商標です


