#DJI Onboard API 문서

last update: 07/28/2015

*버그가 있는 경우 Github issue나 DJI 포럼 혹은 이메일로 알려주세요. 이슈를 수정하기 위해서 pull 요청을 환영한다. 그러나 문서와 관련된 pull 요청은 [문서 스타일](https://github.com/dji-sdk/onboard/issues/8#issuecomment-115976289)을 따라야 한다.*
<br>
<br>

---
DJI는 개발자가 직접 app을 개발할 수 있도록 강력한 2가지 API를 제공한다 : Mobile Device API와 UAV Onboard API이다. Mobile Device API는 DJI Mobile SDK의 일부이며 remote controller에 연결된 iOS나 Android 모바일 장치에서 UAV를 제어 및 감시를 가능하게 한다. UAV Onboard API는 개발자가 유선으로 연결된 시스템(UAR 인터페이스)을 통해 UAV를 제어 및 감시를 가능하게 한다.

이 문서는 onboard API를 소개한다. 2개의 부분으로 구성되어 있다. 바로 시작할 수 있는 onboard API의 주요 컴포넌트에 대한 소개와 모든 개발과 관련된 프로그래밍 가이드이다. 

---

##내용

+ 빠른 시작
  + 소개
    + Onboard API의 핵심 기능
    + 일반 시스템 설명
    + Remote Controller, Onboard API 및 Mobile API
    + 명령 인증 레벨(Command Authorization Levels)
  + [ROS 기반] DJI MATRICE 100의 무선 제어
    + HW 체크리스트
    + SW 체크리스트
    + 설정 순서
+ Onboard API 프로그래밍 가이드
  1. Protocol 상세 설명
    1. Protocol Frame 포맷
    2. Protocol Frame 설명
    3. Protocol Data Field 설명
    4. Session
    5. API Example
  2. 명령 집합 설명(Command Set Explanation)
    1. 명령 집합과 인증 레벨(Command Set and Authorization Level)
    2. 명령 지합(Command Sets)
  3. 비행 제어(Flight Control)에 대한 추가 설명
    1. Coordinate Frames 설명
    2. control mode flag 설명

---

##빠른 시작

먼저 onboard API를 소개하고 일부 용어에 대해서 설명한다. 다음으로 예제 코드로 시작에 필요한 핵심 단계를 살펴본다.

<br>
### 소개

DJI MATRICE 100는 다양한 장비를 보드에 탑재할 수 있도록 넓은 dock을 가지고 있는 특수 용도로 제작된 UAV이다. 착탈이 가능한 배터리, 확장 바, 확장 베이 그리고 추가 XT60 파워 포트는 개발자가 소형 UAV app 시스템을 설계하는데 편리하다. DJI Onboard API는 시리얼 인터페이스를 통해서 MATRICE 100을 직접 제어할 수 있다.

<br>
#### Onboard API의 핵심 기능

+ 신뢰할 수 있는 통신

  32-bit CRC 기반의 세션 기반(Session-based) 링크 프로토콜로 패키지 loss 방지
  
+ 유연한 제어 입력

  위치, 속도, 고도 제어를 포함한 다양한 제어 방법
  
+ 구성 가능한 모니터렁 데이터

  비행 데이터는 아이템과 빈도(frequency)를 설정하여 얻어올 수 있다.
  
+ 자동 Autonomous Application Oriented

  자동 제어와 네비게이션을 돕도록 비행 모드 제어와 비행 데이터를 설계

<br>  
####일반 시스템 설명

MATRICE 100과 여기에 설치된 장치가 핵심 컴포넌트다. onboard 장치는 MATRICE 100 자동 파일럿(N1 Autopilot)에 시리얼 케이블로 연결되어 있다. onboard 장치는 시리얼 통신이 가능하고 AES 암호화를 수행할 수 있는 작은 크기의 컴퓨팅 장치면 가능하다.

DJI N1 PC 보조 소프트웨어는 MATRICE 100 시리얼 포트를 구성하고 MATRICE 100 펌웨어를 업그레이드할 수 있다. 다른 DJI PC기반 소프트웨어와 유사한 도구이다. 이 소프트웨어는 DJI의 새로운 보조 소프트웨어가 제공하지 않는 몇가지 가지고 있다.(펌웨어 업그레이드와 시리얼 포트 구성)

안전과 관련해서 onboard API는 엄청난 권한을 개발자에게 자동화 UAV 시스템 구현을 제공하므로, DJI는 MATRICE 100에 대해서 더욱 엄격한 제어 등록 방법을 사용한다. MATRICE 100을 사용하기 전에, 개발자는 반드시 _dev.dji.com_ 에 개인정보를 등록해야만 하고 MATRICE 100을 활성화해야 한다. DJI 서버는 APP ID와 AES key를 개발자에게 제공할 것이다. onboard 장치와 MATRICE 100 사이의 대부분의 통신은 이 key로 암호화된다. 이것은 활성화 단계 동안 별도로 MATRICE 100에 접근하는 것이다. 활성화와 암호화는 "Active Command Set"에서 상세히 다룬다.

시스템 구조 다이어그램 :
![systemDiagram](Images/systemDiagram.png)

등록과 활성화 단계 다이어그램 :
![registrationDiagram](Images/registrationDiagram.png)

활성화 단계에서 중요한 개념은 DAN(Device Available Number)이다. 다음과 같은 속성을 지닌다 :
+ 각 APP ID는 DAN을 가진다. 이는 동일한 APP ID를 가지고 있는 개발자가 해당 App을 지원하기 위해서 활성화할 수 있는 autopilot의 최대 숫자를 뜻한다.
+ 기본으로 DAN은 새로운 APP ID에 대해서 5개로 제한된다.
+ 활성화 단계동안 autopilot는 DJI 서버에 연결할 때, APP ID의 DAN이 1 증가한다. 만약 DAN이 제한된 수와 같아지면, 새로운 활성화 요청을 허가하지 않는다.
+ 개발자는 APP ID의 DAN 제한을 늘리고 싶다면 _dev.dji.com_ 에 신청해야 한다.

<br>
####Remote Controller, Onboard API & Mobile API

DJI Matrice 드론은 리모트 컨트롤러, onboard 장치, 모바일 장치로 제어되도록 설계되었다. DJI Inspire 1과 Phantom 3를 위한 표준 보조 소프트웨어인 "DJI Pilot"는 해당 플랫폼에서 사용할 수 있다. 또한 DJI Mobile SDK는 플랫폼에 적용되며 해당 플랫폼은 mobile API를 통해 제어할 수 있다.(자세한 내용은 _dev.dji.com_을 방문해서 DJI Mobile SDK에 대해서 알아보자) 3가지 가능한 입력이 있으므로 이것들 사이에 우선순위가 필요하다.

RC는 주요 입력 소스로 설계되었다. 이를 통해 비행 제어가 onboard 장치나 모바일 장치에 권한을 주는 것을 허용한다. RC 컨트롤의 F(function을 뜻함)는 IOC와 API 제어 모드를 포함한 여러 기능을 사용하기 위한 비행 컨트롤러를 제어한다. 다음과 같은 설정을 따르면 비행 컨트롤러는 API 제어 모드로 들어간다:

1. PC 보조 소프트웨어에서 “enable API control” 박스를 체크 (아래 예제에서 상세히 알아보자).
2. IOC 모드는 끄기 (이 설정을 확인하기 위해서는 DJI Pilot 사용)
3. RC 모드 선택 bar를 F에 위치 시키기 (아래 그림 참고)

만약 RC가 비행 컨트롤러가 API 제어를 가능하게 허용하면, onboard API와 모바일 API는 API 요청 기능을 이용해서 제어 권한을 요청할 수 있다. Mobile API는 onboard API보다 더 우선 순위가 높은 제어 권한을 요청할 수 있다. 만약 모바일 API가 제어 우선순위를 가지면 onboard API는 제어 권한을 얻을 수 없다.

![rc](Images/controller.png)

이 문서는 onboard API를 소개하는데 초점을 맞춘다. 모바일 API는 onboard API와 함께 사용하지 않는 것을 가정한다.

**현재 버전에서 하이브리드 컨트롤(모바일 API와 onboard API 모두 사용)는 완벽하게 지원되지 않는다.
**

<br>
####명령 인증 레벨(Command Authorization Levels)

개발자가 _dev.dji.com_ 에 등록할 때, 등록자의 프로그래밍 개발 경험과 요구에 따라서 인증 레벨을 받게 된다. 개발자는 반드시 onboard 장치에 자신의 인증 레벨(app_api_level)을 저장해야 한다. 이 app_api_level은 활성화 단계에서 검사한다.

인증 레벨에 따라서 개발자가 사용할 수 있는 명령이 달라진다.  

+ Level 0, 활성화와 관련된 명령
+ Level 1, 모니터와 비행 제어 이외(카메라나 gimbal 제어, 비행 데이터 모니터링). 이 레벨은 비행의 모션 제어와 직접적으로 관련이 없다.
+ Level 2, 비행 제어. 모션 제어뿐만 아니라 일부 비행 모드 변경 제어 명령을 포함한다. 

향후 onboard API에서 여러 인증 레벨로 더 많은 명령을 제공할 예정이다.
 
<br>
###[ROS 기반] MATRICE 100의 무선 제어

이 예제에는 MATRICE 100을 원격으로 제어하기 위해서 `dji_keyboard_ctrl` 예제 코드를 사용한다. 이 코드는 ROS 패지키인 `keyboardteleopjs`을 기반으로 한다. 사용자가 키보드나 마우스로 MATRICE 100을 제어하는데 익숙해지도록 단순한 HTML GUI를 만든다.
 
<br>
####하드웨어 체크리스트

1. DJI Matrice 개발자 멀티로터 MATRICE 100
2. DJI 시리얼 케이블(MATRICE 100 악세사리에 포함)
3. [DuPont line](http://miniimg.rightinthebox.com/images/384x384/201211/mfbiot1354248218185.jpg) 10-20pcs (전자부품 파는 곳에서 구매 가능)
4. [433(434)](http://www.seeedstudio.com/depot/434Mhz-Wireless-Serial-Transceiver-Module-40-Meters-p-1732.html) 무선 시리얼 트랜시버 모듈 2pcs 
5. [USB to TTL](http://www.adafruit.com/product/954) 시리얼 케이블 1pc <br>
**주의**: PL2303 드라이버는 윈도우/맥에서 USB를 TTL 시리얼 케이블로 사용하는데 필요하다.
6. [5V DC-DC Converter](http://www.adafruit.com/products/1385) <br>
**주의**:  MATRICE 100은 5V 파워를 제공하지 않는다. 따라서 시리얼 트랜시버 모듈은 반드시 외부 DC-DC 컨버터로 전원을 받아야만 한다. 

<br>
####소프트웨어 체크리스트

1. DJI N1 PC 보조 소프트웨어 설치된 윈도우 PC
2. 최신 DJI Pilot가 설치된 모바일 장치. 이 장치는 반드시 인터넷 접속이 되어야만 한다.
3. 우분투 14.04가 설치된 리눅스 PC(혹은 임베디드 장치)와 ROS Indigo(혹은 상위 버전). 예제 코드는 ROS Indigo에서만 테스트 되었다.
4. rosbridge_server ROS 패키지
5. 예제 코드 "dji_sdk"와 "dji_keyboard_ctrl"

<br>
####설정 단계

#####MATRICE 100 준비

MATRICE 100에 전원을 주고 PC에 연결한다. DJI N1 PC 보조 소프트웨어는 API 제어 모드를 사용하기 위해서 사용자가 펌웨어를 업데이트하고 MATRICE 100을 설정해야 한다.

`Basic` 탭에서 개발자는 `enable API control`를 선택하여 원격 컨트롤러 사용을 가능하게 하고 MATRICE가 API 제어와 관련된 접근 기능을 사용할 수 있게 해야한다. 개발자는 `Baud Rate & Message Setting`에서 시리얼 전송 속도와 데이터 패키지 내용을 변경할 수 있다.

![N1UI](Images/N1UI.png)

API 제어 모드를 설정한 후에, 개발자는 API 제어를 가능하기 하기 위해서 리모트 컨트롤러에 모드 선택 바를 가운데 위치(F 위치)로 변경한다.

<br>
#####통신 링크 수립(Establish communication link)

433 트랜시버를 115200 전송속도로 설정한다.(다른 트랜시버는 다른 초기화 단계를 가질 수도 있다.) USB를 TTL 케이블로 하나의 트랜시버를 PC에 연결하고 다른 트랜시버는 DJI 시리얼 케이블을 이용해서 MATRICE의 autopilot에 연결한다. MATRICE 100에 트랜시버는 주의하자. 5V DC-DC 컨버터로부터 전원을 공급받고 MATRICE 100 배터리에서 전원을 끌어다 쓸 수 있기 때문이다.

<br>
#####예제 코드 실행

1. ROS 패지지인 dji_sdk 컴파일 하기
2. roscore 시작시키고 새 터미널에서 rosbridge 서버 시작하기

        roslaunch rosbridge_server rosbridge_websocket.launch
3. 예제 코드내에 있는 런치 파일을 이용해서 dji_sdk_node 시작하기

  다음은 예제 런치 파일이다.
  
  ```xml
  <launch>
  <node pkg="dji_sdk" type="dji_sdk_node" name="dji_sdk_node" output="screen">
  <!-- node parameters -->
  <param name="serial_name" type="string" value="/dev/ttySAC0"/> 
  <param name="baud_rate" type="int" value="115200"/>
  <param name="app_id" type="int" value="<!-- your appid -->"/>
  <param name="app_api_level" type="int" value="<!-- your app level -->"/>
  <param name="app_version" type="int" value="<!-- your app version -->"/>
  <param name="app_bundle_id" type="string" value="<!-- your app bundle id ->"/>
  <param name="enc_key" type="string" value="<!-- your app enc key -->"/> 
  </node>
  </launch> 
  ```
 node 인자는 다음과 같다 :
 
 |Name|Type|Explanation|
 |----|----|-----------|
 |serial_name|String|Serial device's name. Usually it looks like `/dev/ttyUSBx` but different Linux Distribution may have different device name. `ls /dev/` and `dmesg | tail` commands can be used to identify the device name.|
 |baud_rate|Int|The serial port baud rate. It must be the same as the one in MATRICE 100's configuration.|
 |app_id|Int|The APP ID assigned by _dev.dji.com_ server to developer when registration.|
 |app_api_level|Int|The APP API control level assigned by _dev.dji.com_ server to developer when registration.|
 |app_version|Int|Developer assigned application version|
 |app_bundle_id|String|The APP bundle ID assigned by _dev.dji.com_ server to developer when registration.|
 |enc_key|String|The encryption key assigned by dev.dji.com server to developer when registration.|
 
 **주의: 이 명령은 반드시 root 권한으로 실행해야만 한다. (i.e. `sudo su` 먼저).**
     
      sudo su
      roslaunch dji_sdk sdk_demo.launch

4. `sdk_keyboard_demo.html` 수정. 리눅스 머신의 호스트 이름으로 URL 주소를 변경한다. 단일 머신일 경우에는 localhost/127.0.0.1 이며 ROS 여러 머신이 실행 중일 때는 LAN IP로 한다.

    ```c
    function init() {
      // Connecting to ROS.
      var ros = new ROSLIB.Ros({
      url: 'ws://127.0.0.1:9090'
      });
    } 
    ```

5. 웹브라우져에서 `sdk_keyboard_demo.html`를 연다. `rosbridge_server`는 새로 연결된 client를 보여주는 로그를 출력한다. 만약 제대로 출력되지 않는다면 step 4에 있는 연결 설정을 확인한다. html 페이지가 `rosbridge_server`에 연결된 이후, web GUI는 비행 상태를 보여주고 `rostopic`으로 직접 비행 상태를 확인하는 것이 가능하다.

<br>
#####Test communication link
On the web GUI, click button `Activation`. If the communication link is ready to use, MATRICE 100 will acknowledge the GUI. If not, please debug your transceivers and MATRICE 100 settings.

<br>
#####Activate MATRICE 100 to use API
Using DJI Pilot APP to connect the mobile device with the remote controller of MATRICE 100 and make sure the mobile device has Internet access. Then the activation process will be executed automatically after clicking the `Activation` button.

<br>
#####Control MATRICE 100
The web GUI has control buttons as shown below. Moreover, key `W`,`A`,`S`,`D` allows MATRICE move horizontally, `Z`, `C` changes vertical velocity and `Q`,`E` controls yaw movement. Developers can try these functionalities via web GUI but please make sure you have enough testing space first.

The horizontal movement is controlled by angle command associated with button `W`,`A`,`S`,`D`. The angular speed is `5*speed_level`. `speed_level` is an inner variable with default value 1. The value can be changed using key `123456`. Be careful when you are working with high angular speed, MATRICE 100 will accelerate quickly.

<img src="Images/webGUI.png" width="200">

<br>
#####Safety Flight

MATRICE 100 only responses to serial control command when the remote controller’s mode selection bar is at its middle position (F position).  Anytime when user switches the mode away from F position, the API control mode is turned off. We recommend two developers work together during the testing. One developer controls the web GUI, while the other developer hold the remote controller for emergency use.  
If the user wants to switch back to F position and reenter the API mode, onboard application does not need to send control request again to get control authority. If the mode selection bar is already at F position when MATRICE 100 is powered on, then user must switch back and forth to enable API control mode. This mechanism can prevent MATRICE 100 from executing automatic applications without user permission.  
 
---
<br>
## Onboard API Programming Guide

This part discusses the protocol programming when communicating with MATRICE 100.
We recommend developers follow quick start first to run our sample code before reading this programming guide.

<br>
### Protocol Description

#### Protocol Frame Format
   ```
   |<--------------Protocol Frame Header---------------->|<--Protocol Frame Data-->|<--Protocol Frame Checksum-->|
   |SOF|LEN|VER|SESSION|A|RES0|PADDING|ENC|RES1|SEQ|CRC16|          DATA           |            CRC32            |
   ```
 
<br> 
#### Protocol Frame Explanation
<table>
<tr>
  <th>Field</th>
  <th>Byte Index</th>
  <th>Size(bit)</th>
  <th>Description</th>
</tr>

<tr>
  <td>SOF</td>
  <td>0</td>
  <td>8</td>
  <td>Frame start byte, fixed to be 0xAA</td>
</tr>

<tr>
  <td>LEN</td>
  <td rowspan="2">1</td>
  <td>10</td>
  <td>Frame length, maximum length is 1023 (12+1007+4) bytes</td>
</tr>

<tr>
  <td>VER</td>
  <td>6</td>
  <td>Version of the protocol</td>
</tr>

<tr>
  <td>SESSION</td>
  <td rowspan="3">3</td>
  <td>5</td>
  <td>The session ID used during communication</td>
</tr>

<tr>
  <td>A</td>
  <td>1</td>
  <td>Frame Type: <ol start="0"><li>data</li><li>acknowledgement</li></ol></td>
</tr>

<tr>
  <td>RES0</td>
  <td>2</td>
  <td>Reserved bits, fixed to be 0</td>
</tr>

<tr>
  <td>PADDING</td>
  <td rowspan="2">4</td>
  <td>5</td>
  <td>The length of additional data added in link layer. It comes from the encryption process</td>
</tr>

<tr>
  <td>ENC</td>
  <td>3</td>
  <td>Frame data encryption type: <ol start="0"><li>no encryption</li><li>AES encryption</li></ol></td>
</tr>

<tr>
  <td>RES1</td>
  <td>5</td>
  <td>24</td>
  <td>Reserved bits, fixed to be 0</td>
</tr>

<tr>
  <td>SEQ</td>
  <td>8</td>
  <td>16</td>
  <td>Frame sequence number</td>
</tr>

<tr>
  <td>CRC16</td>
  <td>10</td>
  <td>16</td>
  <td>Frame header CRC16 checksum</td>
</tr>

<tr>
  <td>DATA</td>
  <td>12</td>
  <td>---</td>
  <td>Frame data, maximum length 1007 bytes</td>
</tr>

<tr>
  <td>CRC32</td>
  <td>---</td>
  <td>32</td>
  <td>Frame CRC32 checksum</td>
</tr>
</table>

Frame data size varies, 1007 is the maximum length. The index of the CRC32 depends on the length of data field.

<br>
#### Protocol Data Field Explanation

All serial packages exchanged between MATRICE 100 and the onboard device can be classified into three types:
1. Command Package. From the onboard device to MATRICE 100. It mainly contains flight control commands.
2. Message Package. From MATRICE 100 to the onboard device. It contains the autopilot data.
3. Acknowledgement Package (ACK package). From MATRICE 100 to the onboard device. It contains execution results of commands.

<br>
##### Package from the onboard device to MATRICE 100 (Command Package)
```
|<-------Protocol Frame Data------->|
|COMMAND SET|COMMAND ID|COMMAND DATA|
```
|Data Field|Byte Index|Size(byte)|
|----------|----------|----------|
|COMMAND SET|0|1|
|COMMAND ID|1|1|
|COMMAND DATA|2|depends on the exact command|

For the detailed description, please refer to `Command Set Explanation: control commands`

<br>
##### Package from the autopilot to onboard device (Message Package)
```
|<-------Protocol Frame Data------->|
|COMMAND SET|COMMAND ID|COMMAND DATA|
```

|Data Field|Byte Index|Size(byte)|
|----------|----------|----------|
|COMMAND SET|0|1|
|COMMAND ID|1|1|
|COMMAND DATA|2|depends on the exact command|
For the detailed description, please refer to `Command Set Explanation: monitor commands`

<br>
##### Package from the autopilot to onboard device (ACK package)

```
|<-Protocol Frame Data->|
|COMMAND RETURN|ACK DATA|
```

|Data Field|Byte Index|Size(byte)|Description|
|----------|----------|----------|-----------|
|COMMAND RETURN|0|2|The return code of the command result|
|ACK DATA|2|depends on the exact command|Return data|

<br>
#### Session
An important requirement of autonomous control is the communication reliability. We design a "session mechanism" to make sure command packages and ACK packages are exchanged successfully.

When developers compiles a message package in the onboard device program, a session ID should be used depending on the reliability requirement. Different session IDs correspond to different communication channels, i.e. sessions. Onboard API serial link layer has three types of sessions (for brevity, in the following table, we use Sender referring onboard device and Receiver as the autopilot of MATRICE 100).

|Session Mode|SESSION|Description|
|------------|-------|-----------|
|Mode 1|0|Sender do not need acknowledgement|
|Mode 2|1|Sender need acknowledgement, but can tolerate ACK package loss.|
|Mode 3|2-31|Sender wants to make sure the ACK is reliably sent. For these sessions, Receiver saves the sequence number in the command package and send an  ACK package upon receiving it. If ACK package loss happened, Sender may request Receiver again using the same command package with the same sequence number, and Receiver will reply by sending saved acknowledge result. Unless Sender sends a new sequence number, Receiver will not forget the last command acknowledge result|

<br>
#### API Example

Use the following enum to represent the session mode:
```c
enum SESSION_MODE {
  SESSION_MODE1,
  SESSION_MODE2,
  SESSION_MODE3
}
```

And define a callback function to handle the return data of the command:

    typedef void (*CMD_CALLBACK_FUNC)(const void* p_data, unsigned int n_size)

Finally we define 

    unsigned int Linklayer_Send(SESSION_MODE session_mode, const void* p_data, unsigned int n_size, char enc_type, unsigned short ack_timeout, unsigned char retry_time, CMD_CALLBACK_FUNC cmd_callback)

Arguments explained:

|Argument|Description|
|--------|-----------|
|p_data|The start pointer to the datastream|
|n_size|The size of datastream|
|enc_type|Whether this package is encrypted or not|
|ack_timeout|When using session 3, this parameter decides how long to resend command|
|retry_time|When using session 3, this parameter decides how many times to retry|
|cmd_callback|The function pointer to the callback function|

**Note: Here a dummy link layer send interface is defined for demonstration purpose. Since Session Mode 3 is reliable, the communication function interface should contain parameters such as length of timeout and number of resending times.**

<br>
### Command Set Explanation

#### Command Set and Authorization Level

The DJI onboard API has three sets or categories of commands:

|Category|Description|Command Set ID|
|--------|-----------|--------------|
|Activation related|All commands used to activate API|0x00|
|Control related|Commands to control MATRICE 100|0x01|
|Monitoring related|Commands that contains autopilot data|0x02|

Each command set has a unique set ID. All commands belong to one command set have different command ID.

All control commands have an associated authorization level. In the current version, we have set up 5 stable control commands and several unstable commands. All these controls command are level 2 API. In the standard version and future versions, more control commands will be opened up at different authorization levels. A tentative level schedule is shown as follow:

|API Levels|Brief Plan|
|----------|----------|
|0|API activation commands|
|1|Camera and gimbal control commands|
|2|Flight control commands|

<br>
####Command Sets

#####Activation Command Set: 0x00

To activate the API, session ID 2-31 can be used to guarantee ACK packages are returned. All commands in this command set has authorization level 0. Therefore, all users can use these commands to activate MATRICE 100 and debug the connection status. The activation process allows MATRICE 100 to connect to Internet via DJI Pilot and of course a mobile device with Internet access is needed.

<br>
###### Command ID 0x00: Get API version
<table>
<tr>
  <th>Data Type</th>
  <th>Offset</th>
  <th>Size</th>
  <th>Description</th>
</tr>

<tr>
  <td>Request Data</td>
  <td>1</td>
  <td>1</td>
  <td>Arbitrary number</td>
</tr>

<tr>
  <td rowspan="3">Return Data</td>
  <td>0</td>
  <td>2</td>
  <td>Return Code</td>
</tr>

<tr>
  <td>2</td>
  <td>4</td>
  <td>The CRC code of version string</td>
</tr>

<tr>
  <td>6</td>
  <td>32</td>
  <td>SDK version string</td>
</tr>
</table>


Recommended receiving C/C++ struct
```c
typedef struct {
  unsigned short version_ack;
  unsigned int varsion_crc;
  signed char version_number[32];
} version_query_data_t;
```

Set the callback function of getting API version be:
```c
void print_sdk_version(const void* p_data, unsigned int n_size) {
  version_quesry_data_t* p_version = (version_query_data_t*)p_data;
  if (p_version->version_ack == 0) {
    printf("%s\n",p_version->version_name);
  }
}
```

To send getting API version package, we can use the following code:
```c
unsigned char cmd_buf[3];
cmd_buf[0] = 0x00; //command set
cmd_buf[1] = 0x00; //command id
cmd+buf[2] = 0x00; //command data, an arbitrary number as said above
Linklayer_Send(SESSION_MODE3,
                cmd_buf,
                3,
                0,
                200.
                3,
                print_sdk_version
};
```  

Session Mode 3 is used to get API version. After the autopilot receives request and responses, function print\_sdk\_version will executed and print the version information:

    SDK vX.X XXXX

<br>
###### Command ID 0x01: Activate API

<table>
<tr>
  <th>Data Type</th>
  <th>Offset</th>
  <th>Size</th>
  <th>Description</th>
</tr>

<tr>
  <td rowspan="4">Request Data</td>
  <td>0</td>
  <td>4</td>
  <td>app_id, a number obtained when user registers as a developer</td>
</tr>

<tr>
  <td>4</td>
  <td>4</td>
  <td>api_level, authorization level</td>
</tr>

<tr>
  <td>8</td>
  <td>4</td>
  <td>app_ver, the version of onboard application</td>
</tr>

<tr>
  <td>12</td>
  <td>32</td>
  <td>bundle_id, the name of the application, set by developer</td>
</tr>

<tr>
  <td>Return Data</td>
  <td>0</td>
  <td>2</td>
  <td>Return code: <ol start="0"><li>Success</li><li>Invalid parameters</li><li>Cannot recognize encrypted package</li><li>Attempt to activate</li><li>DJI Pilot APP no response</li><li>DJI Pilot APP no Internet</li><li>Server rejected activation attempt</li><li>Insufficient authority level</li></ol></td>
</tr>

</table>

<br>
Recommended sending C/C++ struct
```c
typedef __attribute_((__packed__)) struct { //1 byte aligned
  unsigned int app_id;
  unsigned int ap_api_level;
  unsigned int app_ver;
  unsigned char app_bundle_id[32];
} activation_data_t;
```
**Note: All the structs in the document requires 1 byte alignment (for example using `typedef __attribute__((__packed__))` struct. Developers must make sure their structs are 1-byte aligned.**

Recommended receive C/C++ enum data:

```c
enum ErrorCodeForActivatie {
  errActivateSuccess,
  errActivateInvalidParamLength,
  errActivateDataIsEncrypted,
  errActivateNewDevice,
  errActivateDJIAppNotConnected.
  errActivateDJIAppNoInternet,
  errActivateDJIServerReject,
  errActivateLevelError
};
```

Let the API activation callback function be
```c
void activation_callback(const void* p_data, unsigned int n_size) {

}
```

To send API activation package, we can use following code piece:
```c
unsigned char com_buf[46];
activation_data_t activation_request_data;
//USER TODO...
//activation_request_data.app_id        =0x00;
//activation_request_data.app_api_level =0x00;
//activation_request_data.app_ver       =0x00;
//memset(activation_request_data.app_bundle_id,0,32)
cmd_buf[0] = 0x00; //command set
cmd_buf[1] = 0x01; //command id
memcpy((void*)&cmd_buf[2], (void*)&activation_request_data),sizeof(activation_data_t));
Linklayer_Send(SESSION_MODE3,
                cmd_buf,
                46,
                0,
                200,
                3,
                activation_callback
  );
  
```

Session Mode 3 is used to activate API. After the autopilot receives request and responses, function `activation_callback` will be executed, in which developers can check whether API activation is successful or not.

<br>
###### Command ID 0xFE Data Transparent Transmission (from airborne equipment to mobile device)

The downstream bandwidth from airborne equipment to mobile device is around 8KB/s

|Data Type|Offset|Size|Description|
|---------|------|----|-----------|
|Request Data|0|1~100|User defined data|
|Return Data|0|2|Return code 0: success|

```c
char cmd_buf[10];
cmd_buf[0] = 0x00;
cmd_buf[1] = 0xFE;
memcpy(&cmd_buf[2], "Hello!", 7);
Linklayer_Send(SESSION_MODE3,
                cmd_buf,
                9,
                0,
                200,
                3,
                0
);
```

<br>
##### Control Command: Set 0x01

###### Command ID 0x00: Control Authority Request

|Data Type|Offset|Size|Description|
|---------|------|----|-----------|
|Request Data|0|1|<ul><li>1 = request to get control authority</li><li>0 = request to release control authority</li></ul>|
|Return Data|0|2|Return Code <ul><li>0x0001 = successfully released control authority</li><li>0x0002 = successfully obtained control authority</li><li>0x0003 = control authority failed to change</li></ul>

There are three types of control devices:
1. Remote Controller
2. Mobile Device
3. Onboard Device

The control priority is `Remote Controller > Mobile Device > Onboard Device`. Mobile devices connects to MATRICE via mobile API. A similar control authority request command exists in the mobile API command set. Therefore it is possible that when onboard device request control authority through serial API, control authority is already been granted to the mobile application. So according to the priority list, mobile device will fail to obtain control. Besides, mobile API control request can interrupt ongoing onboard API control.

In the current  version, **hybrid control (using both mobile API and onboard API) is not fully supported.** Developer should take care of the priority issue when developing hybrid control application. A monitoring data `CTRL_DEVICE` can be used to check the control authority (see `Monitor Command Set 0x02`).

0x0003 happens when the mode selection bar of the remote controller is not in F position or control authority is already obtained by mobile application.

Set the callback function for obtaining control authority is:
```c
void get_control_callback(const void* p_data, unsigned int n_size) {

}
```

To send control request package, we can use following code piece 
```c
unsigned char cmd_buf[46];
cmd_buf[0] = 0x01; //command set
cmd_buf[1] = 0x00; //command id
cmd_buf[2] = 0x01; //get control
Linklayer_send(SESSION_MODE3,
                cmd_buf,
                3,
                1,
                200,
                3,
                get_control_callback
);
```
Session Mode 3 is used to obtain control. After the autopilot receives request and responses, function `get_control_callback` will be executed, in which developer can check whether control authority is successfully changed or not.

<br>
###### Command ID 0x01-0x02 Flight Mode Control

To control the flight mode of MATRICE 100, onboard device should use two commands to make sure the mode changing control works properly.

Firstly, the command 0x01 should be sent to trigger the mode changing logic:

<table>
<tr>
  <th>Data Type</th>
  <th>Offset</th>
  <th>Size</th>
  <th>Description</th>
</tr>

<tr>
  <td rowspan="2">Request Data</td>
  <td>0</td>
  <td>1</td>
  <td>Command Sequence Number</td>
</tr>

<tr>
  <td>1</td>
  <td>1</td>
  <td>Request mode<ui><li>1 = request go home</li><li>4 = request auto take off</li><li>6 = request auto landing</li></ui></td>
</tr>

<tr>
  <td>Return Data</td>
  <td>0</td>
  <td>1</td>
  <td>Return code<ui><li>0x0001 = the command is received but rejected</li><li>0x0002 = start to execute the command</li></ui></td>
</tr>

</table>

Once MATRICE 100 receive command 0x01, an immediate ACK package contains either `0x0001` "reject" or `0x0002` "start to execute" will be sent. If the autopilot is already executing a flight mode command, a "reject" command is sent. In normal case, after sending the "start to execute" package, the autopilot will try to change its flight mode and making sure the changing is firmly done. The execution result will be saved.

The second command is a query from the onboard device to obtain the execution result.

|Data Type|Offset|Size|Description|
|---------|------|----|-----------|
|Request Data|0|1|Command Sequence Number|
|Return Data|0|1|Return code<ui><li>0x0001 = query failed, current command is not the query command</li><li>0x0003 = command is executing</li><li>0x0004 = command failed</li><li>0x0005 = command succeed</li></ui>

These two commands, together with the "session mechanism", can guarantee the flight control command is executed and its execution result reaches the onboard device reliably.

<br>
###### Command ID 0x03 Movement Control
**Please read the instructions carefully before sending commands.**

<table>
<tr>
  <th>Data Type</th>
  <th>Offset</th>
  <th>Size</th>
  <th>Description</th>
</tr>

<tr>
  <td rowspan="5">Request Data</td>
  <td>0</td>
  <td>1</td>
  <td>Control mode flag (detailed description in `Additional Explanation for Flight Control`)</td>
</tr>

<tr>
  <td>1</td>
  <td>4</td>
  <td>Roll control value or X-axis control value</td>
</tr>

<tr>
  <td>5</td>
  <td>4</td>
  <td>Pitch control value or Y-axis control value</td>
</tr>

<tr>
  <td>9</td>
  <td>4</td>
  <td>Yaw control value</td>
</tr>

<tr>
  <td>13</td>
  <td>4</td>
  <td>Vertical control value or Z-axis control value</td>
</tr>

<tr>
  <td>Return Data</td>
  <td>0</td>
  <td></td>
  <td>No ACK</td>
</tr>

</table>

<br>
Recommend sending structure in C/C++
```c
typedef __attribute__((__packed__)) struct { // 1 byte aligned
  unsigned char ctrl_flag;
  float roll_or_x;
  float pitch_or_y;
  float yaw;
  float throttle_or_z;
} control_input;
```

**Note: All the structs in the document requires 1 byte alignment (for example using `typedef __attribute__((__packed__))` struct. Developers must make sure their structs are 1-byte aligned.**

Notice that depending on the value of ctrl\_flag, the four control inputs can have different meanings and represent control inputs in either body frame or ground frame. In the "Additional Explanation for Flight Control", body frame, ground frame and ctrl\_flag are elaborated.


**CAUTION！VERY IMPORTANT：control mode has entering conditions：**

- Only when GPS signal is good (health\_flag >=3)，horizontal **position** control (HORI_POS) related control modes can be used.
- Only when GPS signal is good (health\_flag >=3)，or when Gudiance system is working properly (right connection and power provided)，horizontal **velocity** control（HORI_VEL）related control modes can be used.

**About the gps health flag, please read "Command ID 0x00 Message Package"**
**About control modes that contains position and velocity control，please read "Additional Explanation for Flight Control"**


<br>
##### Monitor Command Set: 0x02

###### Command ID 0x00 Message Package

Message Package content and frequency can be configured by DJI N1 assistant software. Each data item in the message package has individual frequency. The maximum message frequency is equal to the message's highest data item update frequency.

<table>
<tr>
  <th>Data Type</th>
  <th>Offset</th>
  <th>Size</th>
  <th>Description</th>
</tr>

<tr>
  <td rowspan="13">Request Data</td>
  <td>0</td>
  <td>2</td>
  <td>Item presence flag<br>bit 0: flag of time stamp<br>bit 1: flag of attitude quaternion<br>bit 2:flag of linear acceleration in ground frame<br>bit 3:flag of linear velocity in ground frame<br>bit 4: flag of angular velocity in body frame<br>bit 5: flag of GPS location, altitude and healthiness<br>bit 6: flag of magnetometer<br>bit 7: flag of remote controller data<br>bit 8: flag of gimbal yaw,pitch,roll<br>bit 9: flag of flight status<br>bit 10:flag of battery info<br>bit 11: flag of control device<br>bit[12:15]: reserved<br><br>Bit with value 1 means the message package contains corresponding data item</td>
</tr>

<tr>
  <td>2</td>
  <td>4</td>
  <td>Time stamp</td>
</tr>

<tr>
  <td>6</td>
  <td>16</td>
  <td>Attitude quaternion item</td>
</tr>

<tr>
  <td>22</td>
  <td>12</td>
  <td>Linear acceleration item</td>
</tr>

<tr>
  <td>34</td>
  <td>12</td>
  <td>Linear velocity item</td>
</tr>

<tr>
  <td>46</td>
  <td>12</td>
  <td>Angular velocity item</td>
</tr>

<tr>
  <td>58</td>
  <td>24</td>
  <td>GPS position, altitude, height and healthiness</td>
</tr>

<tr>
  <td>82</td>
  <td>12</td>
  <td>Magnetometer data</td>
</tr>

<tr>
  <td>94</td>
  <td>10</td>
  <td>Remote controller channels</td>
</tr>

<tr>
  <td>104</td>
  <td>12</td>
  <td>Gimbal yaw, pitch, roll</td>
</tr>

<tr>
  <td>116</td>
  <td>1</td>
  <td>Flight status</td>
</tr>

<tr>
  <td>117</td>
  <td>1</td>
  <td>Battery percentage</td>
</tr>

<tr>
  <td>118</td>
  <td>1</td>
  <td>Control device</td>
</tr>

<tr>
  <td>Return Data</td>
  <td>0</td>
  <td></td>
  <td>No return data</td>
</tr>
</table>

**Note: The first data item is time stamp. Following data items may or may not present in the message packages, so their offsets are *not fixed*. Here we only list the offsets when all data items are sent.**

<br>

Each data item in the message package is explained below:

<table>
<tr>
  <td colspan="5" align="middle"> Data Item List</td>
</tr>
<tr>
  <td>Item Name</td>
  <td>Variables</td>
  <td>Type</td>
  <td>Description</td>
  <td>Default Frequency</td>
</tr>

<tr>
  <td>Time</td>
  <td>time_stamp</td>
  <td>uint32_t</td>
  <td>Time in tick (tick interval 1/600s)</td>
  <td>100Hz</td>
</tr>
<tr>
  <td rowspan="4">Q</td>
  <td>q0</td>
  <td>float32</td>
  <td rowspan="4">Attitude quaternion (From ground to body frame)</td>
  <td rowspan="4">100Hz</td>
</tr>
<tr>
  <td>q1</td>
  <td>float32</td>
</tr>
<tr>
  <td>q2</td>
  <td>float32</td>
</tr>
<tr>
  <td>q3</td>
  <td>float32</td>
</tr>

<tr>
  <td rowspan="3">ACC</td>
  <td>agx</td>
  <td>float32</td>
  <td rowspan="3">Linear acceleration in ground frame</td>
  <td rowspan="3">100Hz</td>
</tr>
<tr>
  <td>agy</td>
  <td>float32</td>
</tr>
<tr>
  <td>agz</td>
  <td>float32</td>
</tr>

<tr>
  <td rowspan="3">VEL</td>
  <td>vgx</td>
  <td>float32</td>
  <td rowspan="3">Linear velocity in ground frame</td>
  <td rowspan="3">100Hz</td>
</tr>
<tr>
  <td>vgy</td>
  <td>float32</td>
</tr>
<tr>
  <td>vgz</td>
  <td>float32</td>
</tr>

<tr>
  <td rowspan="3">W</td>
  <td>wx</td>
  <td>float32</td>
  <td rowspan="3">Angular velocity in body frame</td>
  <td rowspan="3">100Hz</td>
</tr>
<tr>
  <td>wy</td>
  <td>float32</td>
</tr>
<tr>
  <td>wz</td>
  <td>float32</td>
</tr>

<tr>
  <td rowspan="5">POS</td>
  <td>longti</td>
  <td>double</td>
  <td rowspan="2">GPS location</td>
  <td rowspan="5">100Hz</td>
</tr>
<tr>
  <td>lati</td>
  <td>double</td>
</tr>
<tr>
  <td>alti</td>
  <td>float32</td>
  <td>Altitude (measured by barometer)</td>
</tr>
<tr>
  <td>height</td>
  <td>float32</td>
  <td>Height to ground (measured by barometer, may fuse with ultrasonic if sensor added)</td>
</tr>
<tr>
  <td>health_flag</td>
  <td>uint8_t</td>
  <td>GPS healthiness (0-5, 5 is the best condition)</td>
</tr>

<tr>
  <td rowspan="3">MAG</td>
  <td>mx</td>
  <td>float32</td>
  <td rowspan="3">Magnetometer readings</td>
  <td rowspan="3">0Hz</td>
</tr>
<tr>
  <td>my</td>
  <td>float32</td>
</tr>
<tr>
  <td>mz</td>
  <td>float32</td>
</tr>

<tr>
  <td rowspan="6">RC</td>
  <td>roll</td>
  <td>int16_t</td>
  <td>Remote controller roll channel</td>
  <td rowspan="6">50Hz</td>
</tr>
<tr>
  <td>pitch</td>
  <td>int16_t</td>
  <td>Remote controller pitch channel</td>
</tr>
<tr>
  <td>yaw</td>
  <td>int16_t</td>
  <td>Remote controller yaw channel</td>
</tr>
<tr>
  <td>throttle</td>
  <td>int16_t</td>
  <td>Remote controller throttle channel</td>
</tr>
<tr>
  <td>mode</td>
  <td>int16_t</td>
  <td>Remote controller mode channel</td>
</tr>
<tr>
  <td>gear</td>
  <td>int16_t</td>
  <td>Remote controller gear channel</td>
</tr>

<tr>
  <td rowspan="3">GIMBAL</td>
  <td>roll</td>
  <td>float32</td>
  <td>Gimbal roll data</td>
  <td rowspan="3">50Hz</td>
</tr>
<tr>
  <td>pitch</td>
  <td>float32</td>
  <td>Gimbal pitch data</td>
</tr>
<tr>
  <td>yaw</td>
  <td>float32</td>
  <td>Gimbal yaw data</td>
</tr>

<tr>
  <td>FLIGHT_STATUS</td>
  <td>status</td>
  <td>uint8_t</td>
  <td>Flight status</td>
  <td>10Hz</td>
</tr>

<tr>
  <td>BATTERY</td>
  <td>status</td>
  <td>uint8_t</td>
  <td>Battery percentage</td>
  <td>1Hz</td>
</tr>

<tr>
  <td>CTRL_DEVICE</td>
  <td>status</td>
  <td>uint8_t</td>
  <td>Current Control Device<br>0->RC<br>1->APP<br>2->onboard device</td>
  <td>0Hz</td>
</tr>
</table>

<br>
Onboard device can use following code to receive the standard message package sent by the autopilot
```c
typedef struct {
  float q0;
  float q1;
  float q2;
  float q3;
}sdk_q_data_t;

typedef struct {
  float x;
  float y;
  float z;
}sdk_common_data_t;

typedef struct {
  double lati;
  double longti;
  float alti;
  float height;
  short health_flag;
}sdk_gps_height_data_t;

typedef struct {
  signed short roll;
  signed short pitch;
  signed short yaw;
  signed short throttle;
  signed short mode;
  signed short gear;
}sdk_rc_data_t;

typedef struct {
  signed short x;
  signed short y;
  signed short z;
}sdk_mag_data_t;

typedef __attribute_((__packed__)) struct { //1 byte aligned
  unsigned int time_stamp;
  sdk_q_data_t          q;
  sdk_common_data_t     a;
  sdk_common_data_t     v;
  sdk_common_data_t     w;
  sdk_gps_height_data   pos;
  sdk_mag_data_t        msg;
  sdk_rc_data_t         rc;
  sdk_common_data_t     gimbal;
  unsigned char         status;
  unsigned char         battery_remaining_capacity;
  unsigned char         ctrl_device;
}sdk_std_data_t;

#define _recv_std_data(_flag, _enable, _data, _buf, _datalen) \
    if(_flag * _enable) { \
      memcpy ((unsigned char*) &(_data), (unsigned char*)(_buf)+(_datalen), sizeof(_data)); \
      _datalen += sizeof(_data); \
    }

static sdk_std_data_t recv_sdk_std_data = {0};

void recv_std_package (unsigned char* pbuf, unsigned int len) {
  unsigned short *valid_flag = (unsigned short*) pbuf;
  unsigned short data_len = 2;
  
  _recv_std_data(*valid_flag, 0x0001, recv_sdk_std_data.time_stamp,                 pbuf,data_len);
  _recv_std_data(*valid_flag, 0x0002, recv_sdk_std_data.q,                          pbuf,data_len);
  _recv_std_data(*valid_flag, 0x0004, recv_sdk_std_data.a,                          pbuf,data_len);
  _recv_std_data(*valid_flag, 0x0008, recv_sdk_std_data.v,                          pbuf,data_len);
  _recv_std_data(*valid_flag, 0x0010, recv_sdk_std_data.w,                          pbuf,data_len);
  _recv_std_data(*valid_flag, 0x0020, recv_sdk_std_data.pos,                        pbuf,data_len);
  _recv_std_data(*valid_flag, 0x0040, recv_sdk_std_data.mag,                        pbuf,data_len);
  _recv_std_data(*valid_flag, 0x0001, recv_sdk_std_data.rc,                         pbuf,data_len);
  _recv_std_data(*valid_flag, 0x0001, recv_sdk_std_data.gimbal,                     pbuf,data_len);
  _recv_std_data(*valid_flag, 0x0001, recv_sdk_std_data.statis,                     pbuf,data_len);
  _recv_std_data(*valid_flag, 0x0001, recv_sdk_std_data.battery_remaining_capacity, pbuf,data_len);
  _recv_std_data(*valid_flag, 0x0001, recv_sdk_std_data.ctrl_device,                pbuf,data_len);
}
```
**Note: All the structs in the document requires 1 byte alignment (for example using `typedef __attribute__((__packed__))` struct. Developers must make sure their structs are 1-byte aligned.**

<br>

** Further Explanation to The Content **

_alti_ is the result of barometer-IMU fusion in the unit of pressure. While _height_ fuses ultrasonic sensor, barometer and IMU, it means the relative distance in meter from Matrice 100 to the takeoff spot. If the Matrice 100 has no ultrasonic sensor (no Gudiance installed), or it has ultrasonic sensor but its distance to the ground is larger than 3 meters (the measurement of ultrasonic sensor will be unstable at this distance), than _height_ is supported by barometer and IMU only. So developers may notice that Matrice 100 will draft at height above 3 meters, because barometer is very inaccurate indoor. It is very important to remember if developer wants to do indoor flight control.

Since _height_ is a relative distance, so it wouldn't refreash to meaningful value if Matrice does not takeoff after being powered on.

The unit of _lati_, _longti_ in _GPS_ information is **radian**.

The acceleration and angular velocity sent from IMU is processed by signal processing algorithm. We will add flags in the furture version to enable sending raw data out. 

<br>
###### Command ID 0x01: Control Authority Change Notification

Onboard device has the lowers control priority. Its control authority can be taken over by RC and mobile device at any time. Once the control authority changed, a notification message will be sent to the onboard device by the autopilot.

|Data Type|Offset(byte)|Size(byte)|Description|
|---------|------------|----------|-----------|
|Request Data|0|1|Fixed number 0x04|
|Return Data|0|0|No return data|

<br>
###### Command ID 0x02: Data transparent transmission (from mobile device to airborne equipment)

The upstream bandwidth from mobile device to airborne equipment is around 1KB/s

|Data Type|Offset(byte)|Size(byte)|Description|
|---------|------------|----------|-----------|
|Request Data|0|1~100|User defined data|
|Return Data|0|0|No return data|

---
<br>
## Additional Explanation for Flight Control

### Explanation of Coordinate Frames

1. Body Frame

  ![bFrame](Images/axis.png)

2. Ground Frame
  + North - x axis
  + East - y axis
  + Down - z axis

Therefore, in the ground frame, a general definition for craft orientation is North = 0 degree, East = 90 degree, West = -90 degree and South can be either 180 degree or -180 degree.

**Note: The direction of ground frame is not natural for height control. So we adjust the direction of vertical control in order to make height and vertical velocity to be positive upwards, in other words, positive velocity makes MATRICE 100 ascend. This adjustment does not change the direction and the order of the other two axis.**

<br>
### Explanation of ctrl mode flag

To control the spatial movement of MATRICE 100, we split control inputs into three parts: horizontal control, vertical control and yaw control. Each part has several sub modules.

<table>
<tr>
  <th>Category</th>
  <th>Mode</th>
  <th>Explanation</th>
</tr>
<tr>
  <td rowspan="3">Vertical</td>
  <td>VERT_POS</td>
  <td>Control the height of MATRICE 100</td>
</tr>
<tr>
  <td>VERT_VEL</td>
  <td>Control the vertical speed of MATRICE 100, upward is positive</td>
</tr>
<tr>
  <td>VERT_THRUST</td>
  <td>Directly control the thrust (lifting force expressed as 0%-100% percentage) of MATRICE 100</td>
</tr>

<tr>
  <td rowspan="3">Horizontal</td>
  <td>HORI_ATTI_TILT_ANG</td>
  <td>Pitch & roll angle, <b>referenced to either the ground or body frame</b></td>
</tr>
<tr>
  <td>HORI_POS</td>
  <td>Position offsets of pitch & roll directions, <b>referenced to either the ground or body frame</b></td>
</tr>
<tr>
  <td>HORI_VEL</td>
  <td>Velocities on pitches & roll directions, <b>referenced to either the ground or body frame</b></td>
</tr>

<tr>
  <td rowspan="2">Yaw</td>
  <td>YAW_ANG</td>
  <td>Yaw angle referenced to the ground frame</td>
</tr>
<tr>
  <td>YAW_RATE</td>
  <td>Yaw angular rate. It can either be <b>referenced to either the ground frame or the body frame</b></td>
</tr>
</table>

The ctrl mode flag is divided into 8 bits:
<table>
<tr>
  <td rowspan="5">ctrl_mode_flag<br>1byte</td>
  <td>bit[7:6]</td>
  <td>0b00: horizontal angle<br>0b01: horizontal velocity<br>0b10:horizontal position</td>
</tr>
<tr>
  <td>bit[5:4]</td>
  <td>0b00: vertical velocity<br>0b01: vertical position<br>0b10: vertical thrust</td>
</tr>
<tr>
  <td>bit[3]</td>
  <td>0b0: yaw angle<br>0b1: yaw angular rate</td>
</tr>
<tr>
  <td>bit[2:1]</td>
  <td>0b00: horizontal frame is ground frame<br>0b01: horizontal frame is body frame</td>
</tr>
<tr>
  <td>bit[0]</td>
  <td>0b0: yaw frame is ground frame<br>0b1: yaw frame is body frame</td>
</tr>
<table>

`HORI_FRAME` and `YAW_FRAME` can be an arbitrary if the corresponding mode does not need a specify frame. 

By specifying `ctrl_mode_flag`, 14 control modes can be constructed (`ctrl_mode_flag` is represented as an 8-bit binary number here. The bit position with X means that this certain mode doesn't depend on the bit of this position, it can be either 0 or 1. Here "0b" means we represent the flag with binary number, the last 8 bits constructs a 0-255 integer):

|No.|Combinations|Input Data Range<br>(throttle/pitch&roll/yaw)|ctrl_mode_flag|
|---|------------|---------------------------------------------|--------------|
|1|VERT_VEL<br>HORI_ATTI_TILT_ANG<br>YAW_ANG|-4 m/s ~ 4 m/s<br>-30 degree ~ 30 degree<br>-180 degree ~ 180 degree|0b000000XX|
|2|VERT_VEL<br>HORI_ATTI_TILT_ANG<br>YAW_RATE|-4 m/s ~ 4 m/s<br>-30 degree ~ 30 degree<br>-100 degree/s ~ 100 degree/s|0b000010XX|
|3|VERT_VEL<br>HORI_VEL<br>YAW_ANG|-4 m/s ~ 4 m/s<br>-10 m/s ~ 10 m/s<br>-180 degree ~ 180 degree|0b010000XX|
|4|VERT_VEL<br>HORI_VEL<br>YAW_RATE|-4 m/s ~ 4 m/s<br>-10 m/s ~ 10 m/s<br>-100 degree/s ~ 100 degree/s|0b010010XX|
|5|VERT_VEL<br>HORI_POS<br>YAW_ANG|-4 m/s ~ 4 m/s<br>offset in meters (no limit)<br>-180 degree ~ 180 degree|0b100000XX|
|6|VERT_VEL<br>HORI_POS<br>YAW_RATE|-4 m/s ~ 4 m/s<br>offset in meters (no limit)<br>-100 degree/s ~ 100 degree/s|0b100010XX|
|7|VERT_POS<br>HORI_ATTI_TILT_ANG<br>YAW_ANG|0m to height limit<br>-30 degree ~ 30 degree<br>-180 degree ~ 180 degree|0b000100XX|
|8|VERT_POS<br>HORI_ATTI_TILT_ANG<br>YAW_RATE|0m to height limit<br>-30 degree ~ 30 degree<br>-100 degree/s ~ 100 degree/s|0b000110XX|
|9|VERT_POS<br>HORI_VEL<br>YAW_ANG|0m to height limit<br>-10 m/s ~ 10 m/s<br>-180 degree ~ 180 degree|0b010100XX|
|10|VERT_POS<br>HORI_VEL<br>YAW_RATE|0m to height limit<br>-10 m/s ~ 10 m/s<br>-100 degree/s ~ 100 degree/s|0b010110XX|
|11|VERT_POS<br>HORI_POS<br>YAW_ANG|0m to height limit<br>offset in meters (no limit)<br>-180 degree ~ 180 degree|0b100100XX|
|12|VERT_POS<br>HORI_POS<br>YAW_RATE|0m to height limit<br>offset in meters (no limit)<br>-100 degree/s ~ 100 degree/s|0b100110XX|
|13|VERT_THRUST<br>HORI_ATTI_TILT_ANG<br>YAW_ANG|10 ~ 100 (use with precaution)<br>-30 degree ~ 30 degree<br>-180 degree ~ 180 degree|0b001000XX|
|14|VERT_THRUST<br>HORI_ATTI_TILT_ANG<br>YAW_RATE|10 ~ 100 (use with precaution)<br>-30 degree ~ 30 degree<br>-100 degree/s ~ 100 degree/s|0b001010XX|

The input of HORI_POS is a position offset instead a actual position. This design aims to take both GPS flight and vision-based flight into account. If the developer wants to use GPS navigation, the GPS information sent by Matrice 100 can be used to calculate position offset. While in vision-based flight application, developers should have their own positioning device (along with Gudiance or GPS to provide velocity measurement) to do position control. For example, [xuhao1 SDK package](https://github.com/xuhao1/dji_sdk/blob/master/src/modules/dji_services.cpp) realizes a GPS-based position control where target position can be passed as GPS coordinate.

We suggest developers do not user VERT_POS control mode indoor when Matrice 100 does not have Guidance installed or the flight height is larget than 3 meters. Because in indoor environment, barometer can be very inaccurate, the vertical controller may fail to keep the height of Matrice 100. 

**CAUTION！VERY IMPORTANT：control mode has entering conditions：**

- Only when GPS signal is good (health\_flag >=3)，horizontal **position** control (HORI_POS) related control modes can be used.
- Only when GPS signal is good (health\_flag >=3)，or when Gudiance system is working properly (right connection and power provided)，horizontal **velocity** control（HORI_VEL）related control modes can be used.
