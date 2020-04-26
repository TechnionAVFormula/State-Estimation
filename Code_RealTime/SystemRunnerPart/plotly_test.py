import plotly.graph_objects as pigo
import plotly.io as pio


import plotly.graph_objects as go
fig = go.Figure(data=go.Bar(y=[2, 3, 1]))
fig.write_html('first_figure.html', auto_open=True)

x = [2]
y = [3]
scatter = pigo.Scatter(x=x , y=y , mode='markers')
fig = pigo.Figure(data=scatter)
fig.show()