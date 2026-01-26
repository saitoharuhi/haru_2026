src/haru_2026_pkg　→　haru_2026_pkg　にプログラムあります。

## 春ロボプログラムの動かし方
    コマンド２つ起動
    haru_2026に移動　
# cd haru_2026
    source install/setup.bashでrosを動かす
    ps3_nodeとroboware_nodeと別PCでcan_nodeを動かす
# ros2 run haru_2026_pkg ps3_node
    ps3コントローラーの情報を読み取りトピックで次に送信する
# ros2 run haru_2026_pkg roboware_node
    ps3_nodeから送られてきたものをVx,Vy,ωに変換して次に送信する
# ros2 run haru_2026_pkg can_node
    これは別pcで動かす。roboware_nodeから送られてきたのをcanにする
    実行すると、candump can0に操縦データが送られてくる。candump can0を実行する。

## メモ
ls /dev/ttyACM*
canモジュールのポートを調べるコマンドを毎回実行する。抜き差しするとポート番号が変わる可能性あり。
# canを見るには
    sudo modprobe slcan
    sudo modprobe can
    sudo modprobe can_raw
    sudo slcand -o -c -s8 /dev/ttyACM0 can0
    sudo ip link set can0 up
    candump can0
一行ずつコマンドを実行すること。また、ls /dev/ttyACM*で刺さってるか確認
