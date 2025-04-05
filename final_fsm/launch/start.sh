#!/bin/bash -e
enable_ekf=false
record=false
while getopts "er" opt; do
    case $opt in
    e)
        enable_ekf=true
        echo "Enable EKF"
        ;;
    r)
        record=true
        echo "Enable screen recording"
        ;;
    \?)
        echo "Invalid option: -$OPTARG" >&2
        ;;
    esac
done
echo "Start a Tmux session."

# 检查是否已存在 tmux 会话
tmux has-session -t robot_runtime 2>/dev/null
if [ $? != 0 ]; then
    tmux start-server
    uid=$(whoami)

    # 创建一个 tmux 会话 "robot_runtime"
    tmux new-session -d -s robot_runtime

    # 创建主窗口并分割窗格
    tmux new-window -n main
    sleep 1

    tmux split-window -h -t robot_runtime:main
    echo "Created horizontal split at main"
    sleep 1

    tmux split-window -v -t robot_runtime:main.1
    echo "Created vertical split at main.1"
    sleep 1

    tmux split-window -v -t robot_runtime:main.2
    echo "Created vertical split at main.2"
    sleep 1

    tmux split-window -h -t robot_runtime:main.3
    echo "Created horizontal split at main.3"
    sleep 1

    tmux split-window -h -t robot_runtime:main.4

    # 发送命令到各个窗格
    tmux send -t robot_runtime:main.1 "conda activate me5413; export ENABLE_EKF=$enable_ekf; roslaunch me5413_world world.launch" ENTER
    sleep 2
    tmux send -t robot_runtime:main.2 "cd src/final_percep/src; conda activate me5413" ENTER
    sleep 2
    tmux send -t robot_runtime:main.2 "sleep 4 && python visual.py" ENTER
    sleep 2
    tmux send -t robot_runtime:main.3 "export ENABLE_EKF=$enable_ekf; conda activate me5413; sleep 4; roslaunch final_pnc slam_pnc.launch" ENTER
    sleep 2
    tmux send -t robot_runtime:main.4 "conda activate me5413; sleep 7; roslaunch final_fsm fsm.launch" ENTER
    sleep 2

    tmux send -t robot_runtime:main.5 'roscd final_fsm/launch' ENTER
    if [ "$record" = true ]; then
        tmux send -t robot_runtime:main.5 'sleep 7 && ./screen_record.sh' ENTER
    fi
else
    echo "Session 'robot_runtime' already exists. Attaching..."
fi

# 连接到 tmux 会话
tmux attach-session -t robot_runtime
