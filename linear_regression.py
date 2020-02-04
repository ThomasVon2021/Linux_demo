import numpy as np

def calculation_loss(w,b,points):
    totalerror=0
    for i in range(0,len(points)):
        x=points[i,0]
        y=points[i,1]
        totalerror += (w*x+b-y)**2
    return totalerror/float(len(points))

def update_gradient(b_current, w_current, points, learningRate):
    b_gradient = 0
    w_gradient = 0
    N = float(len(points))

    for i in range(0,len(points)):
        x = points[i, 0]
        y = points[i, 1]
        w_gradient += (2/N)*(w_current * x + b_current - y)*x
        b_gradient += (2/N)*(w_current * x + b_current - y)
    #update
    w_new=w_current-learningRate*w_gradient
    b_new=b_current-learningRate*b_gradient
    return [w_new,b_new]

def loop_gradient(points, starting_b, starting_w, learning_rate, num_iterations):
    b=starting_b
    w=starting_w
    for i in range(num_iterations):
        w,b = update_gradient(b,w,np.array(points),learning_rate)
    return [w,b]

if __name__ == '__main__':
    points = np.genfromtxt("data.csv", delimiter=",")
    starting_b=0
    starting_w=0
    learning_rate=0.0001
    num_iterations=10000
    [final_w,final_b]=loop_gradient(points, starting_b, starting_w, learning_rate, num_iterations)
    print("After {0} iterations w = {1}, b = {2}, error = {3}".
          format(num_iterations, final_w, final_b,calculation_loss(final_w, final_b, points))
          )
