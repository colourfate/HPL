---


---

<h1 id="building">building</h1>
<h2 id="特性">特性</h2>
<ol>
<li>使用<a href="https://www.ai-thinker.com/">安信可</a>A9G模块开发的GPS定位器，基本功能是进行定位，然后将定位信息上传到服务器。</li>
<li>使用GPS组合导航技术，将惯性导航信息和GPS定位信息进行融合，得到较高精度的定位信息。</li>
<li>模块充电接口为micro USB接口，并且集成锂电池充电功能，不需使用专门的充电器，只需插上USB接口即可充电。</li>
<li>模块具备USB供电和电池供电自动切换功能，当插入USB电源后，模块将自动从USB接口进行取电，同时对电池进行充电；断开USB电源，模块从电池进行取电。</li>
</ol>
<h2 id="使用方法">使用方法</h2>
<ol>
<li>软件部分使用<a href="https://github.com/Ai-Thinker-Open/GPRS_C_SDK">安信可的SDK</a>进行开发。</li>
<li>PCB图使用Altium Designer绘制，提供原理图和PCB图。</li>
</ol>
<h2 id="更改记录">更改记录</h2>
<ol>
<li>增加惯性导航传感器，能够获取当前模块的姿态、朝向、加速度等信息。</li>
<li>定位和网络通信改为A9G模块，取消了单片机，简化了电路复杂度，电路板尺寸缩小到5*3.2cm。</li>
<li>增加锂电池充电管理芯片。</li>
<li>增加供电自动切换电路。</li>
<li>调整开关、SIM卡插槽和USB接口位置，使其朝向一致。</li>
</ol>

