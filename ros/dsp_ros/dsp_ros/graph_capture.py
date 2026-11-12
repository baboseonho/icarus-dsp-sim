"""화면 없이 ROS 2 노드 그래프를 그린다 (rqt_graph 와 같은 내용).

    from dsp_ros.graph_capture import capture
    capture(node, "out/rosgraph.png")

지금 떠 있는 노드와 토픽을 Graphviz dot 으로 적어 같은 이름의 .dot 과 .png 를 남긴다.

Headless rqt_graph: writes the running nodes and topics as a Graphviz .dot file and a .png.
"""
import subprocess


def capture(node, png_path, skip=("/rosout", "/parameter_events")):
    # 그래프를 묻는 node 자신과 launch, rosbag 의 내부 노드는 뺀다.
    names = [(n, ns) for n, ns in node.get_node_names_and_namespaces()
             if not n.startswith("_") and n != node.get_name() and not n.startswith("launch_ros")
             and not n.startswith("rosbag2")]
    lines = ['digraph G {', ' rankdir=LR; node [fontname="Helvetica", fontsize=11];']
    topics = set()
    for n, ns in names:
        nid = f'"n:{n}"'
        lines.append(f' {nid} [label="{n}", shape=ellipse, style=filled, fillcolor="#d0e4ff"];')
        try:
            pubs = node.get_publisher_names_and_types_by_node(n, ns)
            subs = node.get_subscriber_names_and_types_by_node(n, ns)
        except Exception:  # noqa
            continue
        for t, _ in pubs:
            if t in skip:
                continue
            topics.add(t)
            lines.append(f' {nid} -> "t:{t}";')
        for t, _ in subs:
            if t in skip:
                continue
            topics.add(t)
            lines.append(f' "t:{t}" -> {nid};')
    for t in topics:
        lines.append(f' "t:{t}" [label="{t}", shape=box, fontsize=9, color="#888888"];')
    lines.append("}")
    dot = "\n".join(lines)
    with open(str(png_path).replace(".png", ".dot"), "w") as f:
        f.write(dot)
    subprocess.run(["dot", "-Tpng", "-Gdpi=80", "-o", str(png_path)], input=dot.encode(), check=False)
    return png_path
