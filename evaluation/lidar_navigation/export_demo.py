#!/usr/bin/env python3
"""Render recorded evidence into labelled videos; does not invent sensor motion."""
import argparse
import csv
import json
from pathlib import Path

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont

FONT = '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc'
WIDTH, HEIGHT, FPS = 1280, 720, 20


def text(draw, xy, content, size=22, color='#e6edf3'):
    draw.text(xy, content, font=ImageFont.truetype(FONT, size), fill=color)


def write(writer, frame):
    writer.write(cv2.cvtColor(np.asarray(frame), cv2.COLOR_RGB2BGR))


def writer_for(path):
    writer = cv2.VideoWriter(str(path), cv2.VideoWriter_fourcc(*'avc1'), FPS, (WIDTH, HEIGHT))
    if not writer.isOpened():
        raise RuntimeError('H.264 MP4 encoder unavailable')
    return writer


def paired_video(run, output):
    planning = run / 'planning'
    rows = json.loads((planning / 'paired_planning.json').read_text())
    if len(rows) != 12:
        raise RuntimeError('Paired audit is incomplete')
    writer = writer_for(output / 'lidar_outdoor_paired_demo.mp4')
    try:
        for row in rows:
            frame = Image.new('RGB', (WIDTH, HEIGHT), '#101923')
            draw = ImageDraw.Draw(frame)
            text(draw, (36, 20), '户外录包 · 同源路径规划对照', 30)
            text(draw, (36, 67), f"第 {row['attempt']+1}/12 组 | 共同起终点 | 回放暂停后的地图快照", 21)
            text(draw, (36, 102), f"点云/位姿源时间 {row['source_stamp']:.3f} s | 回放时钟 {row['clock_stamp']:.3f} s", 18)
            for left, name, title in [(40, 'voxel', '原生体素地图'), (660, 'ground', '地面证据候选地图')]:
                result = row['layers'][name]
                data = np.load(planning / result['costmap'])
                grid = data['grid']; origin = data['origin']; res = float(data['resolution'])
                colors = np.full((*grid.shape, 3), [223, 230, 237], np.uint8)
                colors[grid == 255] = [67, 78, 94]
                inflated = (grid > 0) & (grid < 254)
                colors[inflated] = [211, 166, 100]
                colors[grid == 254] = [198, 68, 71]
                tile = Image.fromarray(np.flipud(colors)).resize((450, 450), Image.Resampling.NEAREST)
                frame.paste(tile, (left+50, 158))
                def pixel(p):
                    return (left+50+(p[0]-origin[0])/res/grid.shape[1]*450,
                            158+450-(p[1]-origin[1])/res/grid.shape[0]*450)
                path = [pixel(p) for p in result.get('path', [])]
                if len(path)>1:
                    draw.line(path, fill='#157755', width=4)
                start = pixel(row['start']); goal = pixel(row['goal'])
                radius = .25/res/grid.shape[1]*450
                draw.ellipse((start[0]-radius,start[1]-radius,start[0]+radius,start[1]+radius),outline='#0759ba',width=3)
                draw.ellipse((goal[0]-6,goal[1]-6,goal[0]+6,goal[1]+6),fill='#0759ba')
                text(draw, (left, 128), title, 23)
                code = result.get('error_code')
                status = '接口返回成功' if code == 0 and result.get('status') == 4 else f'失败码 {code}'
                text(draw, (left, 615), f"{status} | 占用 {result['lethal_cells']} 格 | 未知 {result['unknown_cells']} 格", 19)
            text(draw, (40, 651), '红：障碍  棕：膨胀区  深灰：未知  蓝圈：起点及虚拟轮廓  蓝点：目标  绿线：规划路径', 17)
            text(draw, (40, 683), '离散快照，每组展示 4 s；接口成功不等于路径可执行，未知边界与轮廓碰撞仍需核查。', 17)
            if row['attempt'] == 0:
                frame.save(output / 'lidar_outdoor_paired_demo_cover.png')
            for _ in range(FPS*4):
                write(writer, frame)
    finally:
        writer.release()


def simulation_video(run, output):
    with (run / 'trajectory.csv').open() as f:
        data = np.asarray([[float(v) for v in row.values()] for row in csv.DictReader(f)])
    summary = json.loads((run / 'summary.json').read_text())
    xy = np.load(run / 'map_xy.npy')
    times = data[:,0]-data[0,0]
    writer = writer_for(output / 'lidar_nav2_simulation_demo.mp4')
    base = Image.new('RGB',(WIDTH,HEIGHT),'#101923')
    draw = ImageDraw.Draw(base)
    text(draw,(36,20),'Nav2 理想仿真 · 导航闭环轨迹',30)
    text(draw,(36,65),'根据已保存的里程计轨迹重绘，按记录时间播放；不是现场录像。',20)
    origin = (40,125); scale = 50
    def pixel(p):
        return (origin[0]+p[0]*scale,origin[1]+(10-p[1])*scale)
    draw.rectangle((40,125,640,625),fill='#dfe6ed')
    # Same 0.05 m map grid as sim_test.py, including boundary walls.
    for lo,hi in [((0,0),(12,.25)),((0,9.75),(12,10)),((0,0),(.25,10)),((11.75,0),(12,10)),((5.25,3.25),(6,6.75))]:
        a=pixel((lo[0],hi[1]));b=pixel((hi[0],lo[1]));draw.rectangle((*a,*b),fill='#c64447')
    text(draw,(700,150),'来源：Nav2 官方 loopback',24)
    text(draw,(700,195),'理想运动模型与理想点云',22)
    text(draw,(700,252),f"任务结果：{summary['result'].split('.')[-1]}",22)
    text(draw,(700,295),f"轨迹长度：{summary['path_length_m']:.3f} m",22)
    text(draw,(700,338),f"最终目标距离：{summary['final_goal_error_m']:.3f} m",22)
    text(draw,(700,390),'圆形轮廓半径：0.25 m',20)
    text(draw,(700,450),'未验证物理碰撞、打滑、',20)
    text(draw,(700,482),'传感器噪声或真实底盘。',20)
    goal=pixel((9,5));draw.ellipse((goal[0]-7,goal[1]-7,goal[0]+7,goal[1]+7),fill='#0759ba')
    try:
        for t in np.arange(0,times[-1]+2,1/FPS):
            index=min(np.searchsorted(times,t,side='right')-1,len(xy)-1);index=max(index,0)
            frame=base.copy();d=ImageDraw.Draw(frame)
            pts=[pixel(p) for p in xy[:index+1]]
            if len(pts)>1:d.line(pts,fill='#157755',width=4)
            p=pixel(xy[index]);r=.25*scale
            d.ellipse((p[0]-r,p[1]-r,p[0]+r,p[1]+r),fill='#347bc5',outline='#123756',width=2)
            text(d,(40,652),f"记录时间：{min(t,times[-1]):.1f} s | 蓝：虚拟机器人与目标 | 绿：已行驶轨迹",21)
            text(d,(40,687),'该视频展示已验证的软件闭环；最终目标距离不是定位精度。',18)
            if abs(t-10)<.01:frame.save(output/'lidar_nav2_simulation_demo_cover.png')
            write(writer,frame)
    finally:
        writer.release()


def perception_video(run, output):
    directory=run/'perception'
    frames=[]
    for file in sorted(directory.glob('ground_*.npz')):
        with np.load(file) as cloud:
            frames.append((float(cloud['stamp']),file))
    obstacles=[]
    for file in sorted(directory.glob('obstacles_*.npz')):
        with np.load(file) as cloud:
            obstacles.append((float(cloud['stamp']),file))
    if not frames or not obstacles:raise RuntimeError('No recorded perception clouds')
    obstacle_times=np.asarray([p[0] for p in obstacles])
    writer=writer_for(output/'lidar_outdoor_perception_demo.mp4')
    try:
        for stamp,file in frames:
            frame=Image.new('RGB',(WIDTH,HEIGHT),'#101923');d=ImageDraw.Draw(frame)
            text(d,(36,20),'户外录包 · 地面与障碍点云',30)
            text(d,(36,65),f'扫描时间 {stamp:.3f} s | 感知支路抽样输出 | 传感器调平坐标系',20)
            text(d,(40,115),'地面 XY（±15 m）',23)
            text(d,(660,115),'障碍 XY（±15 m，独立抽样）',23)
            array=np.asarray(frame).copy()
            nearest=int(np.argmin(abs(obstacle_times-stamp)))
            point_groups=[]
            with np.load(file) as cloud:point_groups.append((cloud['points'],[72,190,130]))
            delta=float(obstacle_times[nearest]-stamp)
            if abs(delta)<.6:
                with np.load(obstacles[nearest][1]) as cloud:point_groups.append((cloud['points'],[244,118,103]))
            for index,(points,color) in enumerate(point_groups):
                points=points[np.isfinite(points).all(axis=1)]
                left=40 if index==0 else 660
                x=((points[:,0]+15)/30*560+left).astype(int)
                y=(580-(points[:,1]+15)/30*420).astype(int)
                valid=(x>=left)&(x<left+560)&(y>=160)&(y<580)
                array[y[valid],x[valid]]=color
            frame=Image.fromarray(array);d=ImageDraw.Draw(frame)
            for left in [40,660]:d.rectangle((left,160,left+560,580),outline='#8798aa',width=1)
            if len(point_groups[0][0])==0:
                text(d,(90,340),'此帧没有地面点输出',23,color='#f2be79')
            if len(point_groups)==1:
                text(d,(710,340),'此时没有邻近障碍抽样',23,color='#f2be79')
            else:
                text(d,(660,580),f'障碍样本时间差：{delta:+.3f} s（未叠加到地面）',17)
            text(d,(40,612),'绿：地面  红：障碍；人体身份未确认，操作者区域过滤默认关闭。',22)
            text(d,(40,650),'分话题约每秒抽样一帧，分别显示，每帧展示 0.2 s；地面不足时暂停导航观测。',18)
            text(d,(40,686),'这是记录点云的重绘；未展示连续原始扫描，也不是现场摄像视频。',18)
            if file==min(frames,key=lambda item:abs(item[0]-frames[0][0]-50))[1]:frame.save(output/'lidar_outdoor_perception_demo_cover.png')
            for _ in range(4):write(writer,frame)
    finally:
        writer.release()


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--paired',type=Path);p.add_argument('--simulation',type=Path);p.add_argument('--perception',type=Path);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    a.output.mkdir(parents=True,exist_ok=True)
    if a.paired:paired_video(a.paired,a.output)
    if a.simulation:simulation_video(a.simulation,a.output)
    if a.perception:perception_video(a.perception,a.output)
    metadata={}
    for file in a.output.glob('lidar_*demo.mp4'):
        cap=cv2.VideoCapture(str(file));total=int(cap.get(cv2.CAP_PROP_FRAME_COUNT));fps=cap.get(cv2.CAP_PROP_FPS)
        cap.set(cv2.CAP_PROP_POS_FRAMES,max(0,total-1));ok,_=cap.read();cap.release()
        if total<1 or not ok:raise RuntimeError(f'Video cannot be decoded through its final frame: {file}')
        metadata[file.name]={'frames':total,'fps':fps,'duration_s':total/fps,'resolution':[WIDTH,HEIGHT],'last_frame_decoded':ok,'codec':'H.264'}
    (a.output/'lidar_demo_metadata.json').write_text(json.dumps(metadata,indent=2),encoding='utf-8')
    print(metadata)
