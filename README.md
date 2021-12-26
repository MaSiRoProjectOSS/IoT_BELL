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
        node Raspberry as "Raspberry Pi 4"
        node Camera as "カメラ"
    }
    frame tablet_side as "Tablet" {
        node Panel as "タッチパネル"
        node Raspberry as "タブレット\nRaspberry Pi"
        node Camera as "カメラ"
    }

}
rectangle inside_the_store as "店内" {
    person cast as "スタッフ"

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
actor_enter --> Camera : QRコードの提示
Panel <-- Raspberry
TWELITE_MONOSTICK --> server
Camera --> Raspberry

cast <--> server


Raspberry <--> server
server <--> cloud

@enduml

```


## 利用しているハードウェア

| ハードウェア名       | 用途                                 | URL                                                          |
| -------------------- | ------------------------------------ | ------------------------------------------------------------ |
| Raspberry Pi 4       | 携帯サーバー 兼 タブレットの画面表示 | https://www.raspberrypi.com/products/raspberry-pi-4-model-b/ |
| TWELITE® CUE         | ベルのシェイク検知                   | https://mono-wireless.com/jp/products/twelite-cue/index.html |
| MONOSTICK            | TWELITE® CUEのシェイク検知を受信する | https://mono-wireless.com/jp/products/MoNoStick/index.html   |
| Raspberry Pi用カメラ | QRコード表示                         |                                                              |



## License

* QRコードは(株)デンソーウェーブの登録商標です

