# Importing modules...
import numpy as np
import plotly.graph_objects as go

class Viewer: 
    def __init__(self, title, graphical=False, size=5):
        self.title = title
        self.graphical = graphical # Toggle to activate graphical mode
        self.size = size # Change graph dimensions 

        # Create Figure 
        self.figure = go.Figure(
            layout=go.Layout(
                height=700, 
                width=900,
                title=go.layout.Title(text=self.title)
            )
        )

        # Set up layout enviroment
        self.figure.update_layout(
            scene_aspectmode='cube',
            scene = dict(
                xaxis_title='x'*self.graphical,
                yaxis_title='y'*self.graphical, 
                zaxis_title='z'*self.graphical, 
                xaxis=dict(
                    range=[-self.size,self.size],
                    showbackground=self.graphical,
                    showticklabels=self.graphical,
                    showaxeslabels=self.graphical,
                    showgrid=self.graphical,
                    showspikes=self.graphical
                    ),
                yaxis=dict(
                    range=[-self.size,self.size],
                    showbackground=self.graphical,
                    showticklabels=self.graphical,
                    showaxeslabels=self.graphical,
                    showgrid=self.graphical,
                    showspikes=self.graphical
                    ), 
                zaxis=dict(
                    range=[-self.size,self.size],
                    showbackground=self.graphical,
                    showticklabels=self.graphical,
                    showaxeslabels=self.graphical,
                    showgrid=self.graphical,
                    showspikes=self.graphical
                    )
            )
        )

        # Change camera settings
        self.figure.update_layout(
            scene=dict(
                camera=dict(
                    projection=dict(
                        type='orthographic'
                    )
                )
            )
        )

    def add_frame(self, frame, name, color=None):

        # Set default colors
        axis_name_list = ['x', 'y', 'z']
        axis_color_list = ['red', 'green', 'blue']
        origin_color = 'black'

        # Color argument filled
        if color != None:
            axis_color_list = [color]*3 
            origin_color = color 

        self.figure.add_trace(
            go.Scatter3d(
                x=frame.t[0],
                y=frame.t[1],
                z=frame.t[2],
                mode='markers',
                marker=dict(
                    size=3,
                    opacity=0.80,
                    color=origin_color
                ),
                name='{'+name+'}',
                legendgroup='Frames',
                legendgrouptitle_text='Frames',
                showlegend=True
            )
        )

        for axis, axis_color in enumerate(axis_color_list):

            arrow = np.hstack((frame.t, frame.t + frame.R[:,axis].reshape(-1,1))) # Arrow of an axis

            self.figure.add_trace(
                go.Scatter3d(
                    x=arrow[0], 
                    y=arrow[1],
                    z=arrow[2], 
                    mode='lines',
                    line=dict(
                        width=3,
                        color=axis_color
                        ),
                    showlegend=False,
                    name=axis_name_list[axis]+name,
                    hoverinfo = None if self.graphical else 'skip'
                )
            )

    def add_points(self, point, name, color=None):
        self.figure.add_trace(
            go.Scatter3d(
                x=point[0],
                y=point[1],
                z=point[2],
                mode='markers',
                marker=dict(
                    size=3,
                    opacity=0.80,
                    color=color
                ),
                name=name,
                legendgroup='Points',
                legendgrouptitle_text='Points',
                showlegend=self.graphical
            )
        )

    def add_solid(self, vertices, name, color=None):
        self.figure.add_trace(
            go.Mesh3d(
                x=vertices[0],
                y=vertices[1],
                z=vertices[2],
                alphahull=0,
                color=color,
                flatshading=True,
                name=name,
                legendgroup='Objects',
                legendgrouptitle_text='Objects',
                showlegend=self.graphical
            )
        )