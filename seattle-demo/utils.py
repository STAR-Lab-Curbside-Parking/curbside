
import igraph
import xml.etree.ElementTree as ET


def create_graph(net_xml):
    road_network = igraph.Graph(directed=True)

    root = ET.parse(net_xml).getroot()

    # add vertices with names
    # can refer to nodes with their names, instead of id's in most cases
    for child in root.iter('junction'):
        _ = road_network.add_vertex(name=child.get('id'))
    for child in root.iter('edge'):
        if child.get('function') != 'internal':
            _ = road_network.add_edge(child.get('from'), child.get('to'),
                                      name=child.get('id'),
                                      weight=float([kid.get('length') for kid in child.iter('lane')][0]))

    return road_network