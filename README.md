# STM32F767ZIT6_micro-ROS_pub-sub_template
 An STM32 project example using micro-ROS with the STM32F767ZIT6  
このプロジェクトは、STM32でmicro-ROSを使用する際の新規プロジェクト作成時に必要である煩雑な初期設定を簡略化するためのテンプレートです。このテンプレートをコピペして使用することで、面倒な初期設定をスキップし、STM32プロジェクト with micro-ROS を新規作成できます。ただし、micro-ROSを使用する上で必要な各種パッケージ、ツール等（特にdockerなど）は既にインストールされているものとします。

# 使い方
1. このテンプレートを STM32プロジェクトを保存しているディレクトリにcloneする。  
   例）  
   `` cd ~/STM32CubeIDE/workspace_1.15.0/ ``  
   `` git clone https://github.com/hida-yat/STM32f767ZIT6_micro-ROS_pub-sub_template.git ``
2. CubeIDE内のProject Explorer上で右クリックでこのテンプレートをCopy, Pasteを実行し、新しいプロジェクト名でプロジェクトを新規作成する。
3. 新規作成したプロジェクトの中の ``micro-ROS_template.ioc`` を ``[新規作成したプロジェクト名].ioc`` にRenameする。
4. Run >> Debug Configurationを開き、Main / C/C++ Aplication の項目について、 ``Debug/micro-ROS_template.elf`` を参照するようになっている。そのため、「Browse…」ボタンを押し起動したファイラーで、プロジェクトのディレクトリまで移動し、Debugディレクトリの ``[新規作成したプロジェクト名].elf`` を選択し、ビルドできることを確認する。

#  publlsherとsubscriberについて
## publisher
* 使用例として、エンコーダを使用して ``/f767zi_encoder`` トピックとしてpublishしている。メッセージ型は、stdmsgs/msg/Int32である。
* エンコーダのA相、B相の信号には、TIM2のEncoder Modeを使用しており、それぞれPA0, PA1ピンを使用している。

## subscliber
* 使用例として、　``/cmd_vel`` トピックをsubscribeして、linear.xが0以上であればSTM32F767ZIT6のオンボードLEDであるLD1が点灯する。

# micro_ros_agentの起動例
 ``ros2 run micro_ros_agent micro_ros_agent serial -b 115200 --dev /dev/ttyACM0`` 

# おまけ
## /f767zi_encoderトピックの確認の一例
 ``ros2 topic echo /f767zi_encoder`` 
## /cmd_velトピックのpublishの仕方の一例
 ``ros2 run teleop_twist_keyboard teleop_twist_keyboard`` でIキーを押すと、linear.xが0.5となりLD1が点灯、Hキーを押すとlinear.xが0になり消灯する。
