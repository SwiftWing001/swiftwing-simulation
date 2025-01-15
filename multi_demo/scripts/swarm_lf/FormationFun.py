import numpy as np


def CircleForm(follower_amount, distance):
    r = distance / (2. * np.pi) * follower_amount
    angle = np.linspace(0., follower_amount-1, follower_amount) * (2. * np.pi) / follower_amount
    offset_list = np.array([np.sin(angle), np.cos(angle)]) * r
    return offset_list.T

def RectangleCompact(follower_amount, distance):
    column_amount = np.floor(np.sqrt(follower_amount + 1.))
    row_amount = np.ceil((follower_amount + 1.) / column_amount)
    
    form_list = np.zeros([0, 2])
    for i in range(int(column_amount)):
        for j in range(int(row_amount)):
            if i * row_amount + j >= (follower_amount + 1.):
                print("amount: ", i * row_amount + j)
                return form_list
            if i==0 and j==0:
                continue
            coordinate = np.array([i, -j]) * distance
            form_list = np.append(form_list, [coordinate], axis=0)
    return form_list
                



if __name__ == "__main__":
    # form = CircleForm(5, 30.)
    form = RectangleCompact(8, 30.)
    print(form)

    import plotly.graph_objects as go

    fig = go.Figure()
    fig.add_trace(go.Scatter(x=form[:, 0], y=form[:, 1], mode="markers+lines"))
    fig.show()

    
