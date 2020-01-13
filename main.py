import matplotlib.pyplot as plt
import os
from bs4 import BeautifulSoup


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
    tmax = int(tmax)
    m = int(m)
    b = float(b)
    y0 = int(y0)
    v0 = int(v0)
    target = int(target)
    v_opt = float(v_opt)
    f_max = int(f_max)
    use_r_h = bool(use_r_h)
    n = int((tmax - t0) // dt)
    if use_r_h:
        r_h = y0
    else:
        r_h = target
        v_opt = 0.0

    rhs = [r_h, r_h + v_opt * dt]
    t = t0
    f_add = m * g
    t0, t1 = 0, dt
    y1 = y0 + v0 * dt
    ts, ys = [t0, t1], [y0, y1]
    fs = [0, 0]
    vs = [v0, v0]
    pid = PID(50, 0.05, 50)
    for _ in range(n):
        t += dt
        if use_r_h:
            if r_h < target:
                r_h += v_opt * dt
                if r_h >= target: use_r_h = False
            if r_h > target:
                r_h -= v_opt * dt
                if r_h <= target: use_r_h = False
        rhs += [r_h]
        e = r_h - y1
        f = pid(e, dt)
        l = max(min(f, f_max) + f_add, 0) - m * g
        y = (m * (2 * y1 - y0) + l * (dt ** 2) - b * abs(y1 - y0) * (y1 - y0)) / m  # drag b is bidirectional
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


from flask import Flask, render_template, request
import base64
from flask import send_file

app = Flask(__name__)
app.config['TEMPLATES_AUTO_RELOAD'] = True
app.config['SEND_FILE_MAX_AGE_DEFAULT'] = 0


# No caching at all for API endpoints.
@app.after_request
def add_header(response):
    # response.cache_control.no_store = True
    response.headers['Cache-Control'] = 'no-store, no-cache, must-revalidate, post-check=0, pre-check=0, max-age=0'
    response.headers['Pragma'] = 'no-cache'
    response.headers['Expires'] = '-1'
    return response


@app.route('/', methods=['POST', 'GET'])
def student():
    if request.method == 'POST':
        # tmax, m = 750, b = 180.9, y0 = 0, v0 = 0, target = 100, v_opt = 3.5, f_max = 3000, use_r_h = True
        result = request.form
        my_dict = dict(result.items())
        plot1(0, **my_dict)
        cur_path = os.path.dirname(__file__)
        new_path = os.path.join(cur_path, 'templates/index.html')
        new_path1 = os.path.join(cur_path, 'static/images/graphon.png')
        fig1 = plt.gcf()
        plt.show()
        plt.draw()
        fig1.savefig(new_path1, dpi=100)
        html = open(new_path)
        soup = BeautifulSoup(html, "html.parser")
        print(soup)
        extraSoup = BeautifulSoup('<img  id="imgs" src="static/images/graphon.png">', "html.parser")
        tag = soup.find(class_='my_img')
        try:
            blah = soup.find(id='imgs')
            blah.extract()
        except:
            print("Niema obrazka:(")
        tag.insert(1, extraSoup.img)
        with open(new_path, 'w') as f:
            f.write(str(soup))
        # return render_template("test.html", result=result)
        return render_template('index.html')
    if request.method == 'GET':
        return render_template('index.html')

if __name__ == "__main__":
    app.run()
