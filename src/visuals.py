import plotly.express as px
import plotly.graph_objects as go
import numpy as np

def visualize_graph(n_nodes, positions, children):
    fig = go.Figure()

    # Add lines from parent to children
    for i in range(n_nodes):
        for child in children[i]:
            fig.add_trace(go.Scatter(
                x=[positions[i, 0], positions[child, 0]],
                y=[positions[i, 1], positions[child, 1]],
                mode='lines',
                line=dict(color='gray', width=2),
                showlegend=False
            ))

    # Add nodes on top
    scatter = go.Scatter(
        x=positions[:, 0], 
        y=positions[:, 1], 
        mode='markers',
        text=[str(i) for i in range(n_nodes)],
        textposition='middle center',
        marker=dict(size=5, color='lightblue', line=dict(color='black', width=2))
    )
    fig.add_trace(scatter)

    fig.update_layout(
        title="Random Tree Visualization",
        xaxis_title="X",
        yaxis_title="Y",
        showlegend=False
    )
    return fig

def animate_trajectories(fig, node_positions, trajectories, node_colors=None, highlight=None):        
    trajectories = np.asarray(trajectories)

    n_nodes = node_positions.shape[0]
    n_trajectories = trajectories.shape[0]
    n_frames = trajectories.shape[1]
    frames = []
    trace = len(fig.data)

    # Use provided colors or default to trajectory index
    if node_colors is None:
        node_colors = np.full([n_frames, n_nodes], 'rgba(0, 0, 0, 0)', dtype=object)
    else:
        node_colors = np.asarray(node_colors, dtype=object)

    n_colors = len(px.colors.qualitative.Set3)
    for i in range(n_trajectories):
        color = px.colors.qualitative.Set3[i % n_colors]
        node_colors[np.arange(n_frames), trajectories[i]] = color

    for i in range(n_frames):
        x = node_positions[:, 0]
        y = node_positions[:, 1]

        node_size = np.full([n_nodes], 8).astype(int)
        if highlight is not None:
            node_size[trajectories[highlight, i]] = 14
        
        frame_colors = node_colors[i]
        frames.append(
            go.Frame(
                data=go.Scatter(
                    x=x, y=y,
                    hoverinfo='none',
                    mode='markers',
                    marker=dict(size=node_size, color=frame_colors, opacity=1)
                ),
                traces=[trace]
            )
        )

    fig.add_trace(frames[0].data[0])
    fig.update(frames=frames)

    fig.update_layout(updatemenus=[dict(
            type="buttons",
            showactive=False,
            buttons=[dict(label="Play",
                          method="animate",
                          args=[None, {"frame": {"duration": 500, "redraw": True}, 
                                        "fromcurrent": True, 
                                        "transition": {"duration": 0}}])]
        )])

def get_node_color_for_agent(agent_id, n_nodes, explored_nodes):
    n_timesteps = len(explored_nodes[agent_id])
    node_colors = np.full((n_timesteps, n_nodes), 'rgba(0, 0, 0, 255)', dtype=object)

    for t, explored_list in enumerate(explored_nodes[agent_id]):
        for node_id in explored_list:
            node_colors[t, node_id] = 'rgba(100, 255, 100, 255)'
    return node_colors