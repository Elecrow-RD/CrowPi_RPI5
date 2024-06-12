from gpiozero import Button, LED
from time import sleep

# 定义按键矩阵的行和列引脚
row_pins = [27, 22, 5, 6]
col_pins = [25, 26, 19, 13]

# 定义按键矩阵的键值映射
key_map = [
    ['1', '2', '3', '4'],
    ['5', '6', '7', '8'],
    ['9', '10', '11', '12'],
    ['13', '14', '15', '16']
]

# 初始化行输入和列输出引脚
rows = [Button(pin, pull_up=True) for pin in row_pins]
cols = [LED(pin) for pin in col_pins]

# 持续扫描按键
try:
    while True:
        # 逐个设置列输出引脚为低电平，然后检测行输入引脚的状态
        for col in cols:
            col.off()
            for row in rows:
                if row.is_active:
                    # 检测到按键按下
                    row.wait_for_release()
                    print("Key press:", key_map[rows.index(row)][cols.index(col)])
                    break
            col.on()
        sleep(0.1)

except KeyboardInterrupt:
    print("end")

finally:
    # 清理GPIO状态
    for row in rows:
        row.close()
    for col in cols:
        col.close()