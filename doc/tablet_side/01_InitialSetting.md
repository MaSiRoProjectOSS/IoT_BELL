# Tablet side

<link rel="stylesheet" href="https://github.com/MaSiRo-Project-OSS/IoT_BELL/blob/develop/doc/style.less?raw=true">

<div style="float: right">作成日:2021/12/26</div>

## 初期設定


### 起動用SDカードの作成

1. "Raspberry Pi"の起動用SDカードを作成します。
作成には「[Raspberry Pi Imager](https://www.raspberrypi.com/software/)」を使用しますので公式よりダウンロードしてください。
2. 「Raspberry Pi Imager」のOperating Systemには、"Other geneal purpose OS"->"Ubuntu"->"Ubuntu server 20.04.3 LTS(RPi 2/3/4/400)"を選択し、SDカードに書き込んでください。
    * ROSを動かすため2021/12/26時点でサポートしている「Ubuntu server 20.0.4.3 LTS」を選択している。

### Raspberry Piの設定

起動時のUbuntu serverの初期ユーザ名と初期パスワードは下記になります。**後述する設定で変更**してください。

| 項目           | 値     |
| -------------- | ------ |
| 初期ユーザ名   | ubuntu |
| 初期パスワード | ubuntu |

#### ホスト名を変更する

```bash
CHANGE_HOSTNAME=変更したいホスト名
hostnamectl set-hostname $CHANGE_HOSTNAME
```

#### ログインアカウントを設定する

##### 管理者用のアカウント作成

```bash
USER_NAME=作成したい管理者用アカウント名(小文字であること)

## ユーザー名を追加
sudo adduser $USER_NAME

## 権限付与
sudo gpasswd -a $USER_NAME sudo
sudo gpasswd -a $USER_NAME dialout
sudo gpasswd -a $USER_NAME video

## パスワード変更
su $USER_NAME
passwd
```

##### 店内稼働用アカウントの作成

```bash
USER_NAME=作成したい店内稼働用アカウント名

## ユーザー名を追加
sudo adduser $USER_NAME(小文字であること)

## 権限付与
sudo gpasswd -a $USER_NAME dialout
sudo gpasswd -a $USER_NAME video

## パスワード変更
su $USER_NAME
passwd
```

##### (option)デフォルトユーザの削除

上記のアカウントを作成しログインが確認できたならば、**初期アカウントの削除することをお勧めします**。

```bash
## デフォルトユーザの削除
sudo userdel -r ubuntu
```

#### wifiの設定

ワイヤレスで環境を構築するため```/etc/netplan/50-cloud-init.yaml``` に下記のWifi設定をする。

* 後でGUIのインストールするのでLANケーブルを使って接続できる環境があるならばGUIで設定後に作業してもOKです。

```text
/etc/netplan/50-cloud-init.yaml
network:
    version: 2
    ethernets:
        eth0:
            dhcp4: true
            match:
                macaddress: <MAC ADDRESS>
            set-name: eth0
    # ここから下の行を追加
    wifis:
        wlan0:
            optional: true
            dhcp4: true
            access-points:
                "wifi id":
                    password: "wifi password"
```


#### ディスプレイマネージャーのインストール

```bash
apt update
apt upgrade

# ディスプレイマネージャーlightdmのインストール
sudo apt -y install lightdm

# 起動時にGUIを起動するように設定
sudo systemctl set-default graphical.target
sudo reboot
```

#### Raspberry Pi用toolのインストール

GPIO操作するため下記コマンドでtoolsをインストールする

```bash
sudo echo "deb http://archive.raspberrypi.org/debian/ buster main" >> /etc/apt/sources.list
sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys 7FA3303E
sudo apt update
sudo apt install -y raspi-config

sudo apt install -y libv4l-dev v4l-utils
sudo apt install -y libraspberrypi-bin
sudo apt install -y linux-tools-raspi
sudo apt install -y pigpio-tools
```

##### Raspberry Piの設定を行う

下記のコマンドから日時設定やキーボード設定などを行ってください。

```bash
sudo raspi-config
```


#### ROS2のインストール

ROS2の[Installing ROS 2 on Ubuntu Linux](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Binary.html)を基にインストールしてください。

* ROSのインストール方法については時期によって手順が変わるため最新のサイトをご確認ください。




### その他

* SSHの接続環境を作成しパスワードで入れないようにするのがおススメです。
* gitにアクセスする場合はアカウント設定を行ってください。
```bash
git config --global user.email "メールアドレス"
git config --global user.name "ユーザ名"
```


---

## システム改変のため保留項目



### スクリーンセイバーをOFFにする

画面がOFFにならないようにスクリーンセイバーをインスールし無効にする。

```bash
sudo apt-get install xscreensaver
```

上記の「xscreensaver」をインスールしたら```/etc/lightdm/lightdm.conf```に下記の内容を追記する。

```text
[SeatDefaults]
xserver-command=X -s 0 -dpms
```


### カメラの設定

カメラケーブルでカメラの接続設定を行う。


カメラを認識させるために```/boot/firmware/config.txt```に下記の設定を追記する

* config.txtに"usercfg.txt"に追加してくださいと追記されるが動作しないとのことなので"config.txt"に記載する。

```text
[all]
start_x=1
gpu_mem=128
```

上記の設定が完了したら```sudo reboot```で再起動を行う。
再起動後```/dev/video0```が追加される。

下記のコマンドでカメラが認識しているか確認する。
認識しない場合はカメラケーブルの向きを間違えたなどが考えられる。

```bash
v4l2-ctl --list-devices
```

下記のコマンドで３秒後に撮影出来るかテストする

```bash
raspistill -t 3000 -o ~/image.jpg
```