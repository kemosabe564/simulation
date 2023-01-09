import numpy as np
import matplotlib.pyplot as plt

if(__name__ == '__main__'):
    filename = open("noise_measurement\\noise_log.txt", "r")
    x = []
    y = []
    for oneline in filename:
        onewords = oneline.split()
        if onewords[0] == "position:":
            flag = True
        elif onewords[0] == "orientation:":
            flag = False
        if onewords[0] == "x:" and flag == True:
            x.append(float(onewords[1]))
    print(x)
    
    print(np.var(x))
    t = np.arange(0, len(x), 1)
    # print(t)
    plt.figure(figsize=(8, 6), dpi = 80)
    plt.plot(t, x, '.')
    plt.axhline(y = np.mean(x), ls = ":", c = "red")
    
    plt.title("camera noise in x axis") 
    plt.xlabel('time'); plt.ylabel('value'); 
    plt.savefig('pics/camera noise in x axis.png')  
    plt.show() 
    