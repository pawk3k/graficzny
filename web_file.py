import matplotlib.pyplot as plt
import matplotlib.pyplot as plt, mpld3
import base64
import os
from io import BytesIO


class PID:
    def __init__(self, Kp, Ki, Kd):
        self.Kp, self.Ki, self.Kd = Kp, Ki, Kd
        self.e_sum = 0
        self.last_e = 0

    def __call__(self, e, dt):
        self.e_sum += e * dt
        v = self.Kp * e + self.Ki * self.e_sum + self.Kd * (e - self.last_e) / dt
        self.last_e = e
        return v


dt = 0.01
g = 9.81
def control_system(t0, tmax, m=750, b=180.9, y0=0, v0=0, target=100, v_opt=3.5, f_max=3000, use_r_h=True):
    global dt, g
    n = int((tmax-t0) // dt)
    r_h = 0  # reference height
    t = t0
    f_add = m*g
    t0, t1 = 0, dt
    y1 = y0 + v0 * dt
    ts, ys = [t0, t1], [y0, y1]
    rhs = [0, v_opt*dt]
    fs = [0, 0]
    vs = [v0, v0]
    pid = PID(50, 0.05, 50)
    for _ in range(n):
        t += dt
        if use_r_h:
            if r_h < target:
                r_h += v_opt*dt
        else:
            r_h = target
        rhs += [r_h]
        e = r_h - y1
        f = pid(e, dt)
        l = max(min(f, f_max) + f_add, 0) - m*g
        y = (m*(2*y1 - y0) + l*(dt**2) - b*abs(y1-y0)*(y1-y0)) / m  # drag b is bidirectional
        y = max(y, 0)
        ts += [t]
        ys += [y]
        y0 = y1
        y1 = y

    return ts, ys, rhs, fs, vs



def plot1(t0, tmax, **kwargs):
    ts, ys, rhs, fs, vs = control_system(t0, tmax, **kwargs)
    plt.title("Wysokość balonu w funkcji czasu")
    plt.xlabel("t [s]")
    plt.ylabel("y(t) [m]")
    plt.plot(ts, rhs, ":", label="wysokość optymalna")
    plt.plot(ts, ys, label="wysokość rzeczywista")
    plt.legend()

def plot(t0, tmax, **kwargs):
        ts1, ys1, rhs = control_system(t0, tmax, **kwargs)

        le = ", ".join([k + "=" + str(kwargs[k]) for k in kwargs.keys()])
        plt.xlabel("t")
        plt.ylabel("h(t)")
        plt.plot(ts1, rhs, ":")
        plt.plot(ts1, ys1, label=le)
        plt.legend()


import requests
# from flask.ext.bootstrap import Bootstrap
from flask import Flask, render_template, request
app = Flask(__name__)
app.config['TEMPLATES_AUTO_RELOAD'] = True
@app.route('/')
def student():
   return render_template('index.html')

@app.route('/result',methods = ['POST', 'GET'])
def result():
   if request.method == 'POST':
      result = request.form
      # print(result.keys())
      # t0 = result.get("t0")
      # tmax = result.get("tmax")
      # m_min = result.get("m_min")
      # m_max = result.get("m_max")
      # print(t0)
      # print(tmax)
      # cur_path = os.path.dirname(__file__)
      #
      # new_path = os.path.join(cur_path, 'templates/test.html')
      # new_path1 = os.path.join(cur_path, 'static/images/graphon.png')
      # plt.title("Mała masa balonu vs duża")
      # plot(int(t0), int(tmax), m=int(m_min))
      # plot(int(t0), int(tmax), m=int(m_max))
      # fig1 = plt.gcf()
      # plt.show()
      # plt.draw()
      #
      # fig1.savefig(new_path1, dpi=100)
      # html = 'Some html head32' + '<img src=\'' + 'static/images/graphon.png\'>' + 'Some more html'
      # with open(new_path, 'w') as f:
      #     f.write(html)
      return render_template("result.html",result = result)



if __name__ == "__main__":
    app.run()

    # cur_path = os.path.dirname(__file__)
    # total_path = '/templates/static/images/graph.png'
    #
    # new_path = os.path.join(cur_path, 'templates/test.html')
    # new_path1 = os.path.join(cur_path, 'templates/static/images/graph.png')
    # plt.title("Mała masa balonu vs duża")
    # plot(0, 200, m=750)
    # plot(0, 200, m=7500)
    # # plt.show()
    # fig1 = plt.gcf()
    # plt.show()
    # plt.draw()
    # fig1.savefig(new_path1)
    # html = 'Some html head1' + '<img src=\'/static/images/fig.png\'>' + 'Some more html'
    # with open(new_path, 'w') as f:
    #     f.write(html)
    #
    # plt.title("Różne docelowe wysokości")
    # plot(0, 200, target=25)
    # plot(0, 200, target=100)
    # plot(0, 200, target=500)
    # plt.show()
    #
    # plt.title("Opór powietrza vs brak oporu")
    # plot(0, 200, b=180.9)
    # plot(0, 200, b=0)
    # plt.show()
    #
    # plt.title("Zmienna wartość docelowa vs stała")
    # plot(0, 1000, target=2000, use_r_h=True)
    # plot(0, 1000, target=2000, use_r_h=False)
    # plt.show()


