# Tablet side

<link rel="stylesheet" href="https://github.com/MaSiRo-Project-OSS/IoT_BELL/doc/style.less?raw=true">

## 初期設定

### 起動用SDカードの準備

「Raspberry Pi Imager」をつかって「Ubuntu server 20.0.4.3 LTS」をインストールする。

* ROSを動かすため2021/12/26時点でサポートしている「Ubuntu server 20.0.4.3 LTS」を選択している。


### Raspberry Piの設定

「Raspberry Pi Imager」を使ってSDカードを作成した場合は、2021/12/26時点では下記になります。
**後述する設定で変更**してください。

| 項目         | 値     |
| ------------ | ------ |
| 初期ユーザ名 | ubuntu |
| 初期パス     | ubuntu |

#### ホスト名を変更する

```bash
CHANGE_HOSTNAME=変更したいホスト名
hostnamectl set-hostname $CHANGE_HOSTNAME
```

#### ログインアカウントを設定する

##### 管理者用のアカウント作成

```bash
USER_NAME=作成したい管理者用アカウント名

## ユーザー名を変更
sudo adduser $USER_NAME

## 権限付与
sudo gpasswd -a $USER_NAME sudo
sudo gpasswd -a $USER_NAME dialout

## (option)デフォルトユーザの削除
sudo userdel -r ubuntu
```

##### 店内稼働用アカウントの作成

```bash
USER_NAME=作成したい店内稼働用アカウント名

## ユーザー名を変更
sudo adduser $USER_NAME

## 権限付与
sudo gpasswd -a $USER_NAME dialout

## (option)デフォルトユーザの削除
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
sudo apt -y install lightdm
```

#### ROS2のインストール

ROS2の[Installing ROS 2 via Debian Packages](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html)を基にインストールしてください。

* ROSのインストール方法については時期によって手順が変わるため最新のサイトをご確認ください。

#### その他

SSHの接続環境を作成しパスワードで入れないようにするのがおススメです。


