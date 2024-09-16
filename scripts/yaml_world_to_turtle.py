#!/usr/bin/env python3
import os
import yaml
from rdflib import Graph, Namespace, Literal, RDF, URIRef
from rdflib.namespace import XSD, RDFS, OWL

from ament_index_python import get_package_share_directory

# Namespaces
CORP = Namespace("http://www.inf.ufrgs.br/phi-group/ontologies/cora.owl#")
SKIROS = Namespace("http://rvmi.aau.dk/ontologies/skiros.owl#")
RDFLIB = Namespace("http://www.w3.org/2000/01/rdf-schema#")
ROBI = Namespace("http://rvmi.aau.dk/ontologies/robi.owl#")
RPARTS = Namespace("http://www.inf.ufrgs.br/phi-group/ontologies/RParts.owl#")

# Parse YAML
def load_yaml(filepath):
    """Load YAML data from a file."""
    try:
        with open(filepath, 'r') as file:
            return yaml.safe_load(file)
    except Exception as e:
        print(f"Error loading YAML file: {e}")
        raise

# Helper function to generate unique IDs
class IDGenerator:
    """Generates unique IDs for RDF entities."""
    def __init__(self):
        self.counter = 1

    def get_next(self):
        id_number = self.counter
        self.counter += 1
        return id_number

# Create Turtle for the scene (always Scene-0)
def create_scene_ttl(graph, world_number: int):
    """Create Turtle for the scene."""
    scene_uri = SKIROS["Scene-0"]
    graph.add((scene_uri, RDF.type, SKIROS.Scene))
    graph.add((scene_uri, RDF.type, OWL.NamedIndividual))
    graph.add((scene_uri, RDFS.label, Literal(f"Scene World {world_number}")))
    graph.add((scene_uri, SKIROS.FrameId, Literal("map", datatype=XSD.string)))
    return scene_uri

# Create Turtle for the robot with the specified attributes and a gripper
def create_robot_ttl(graph, robot_data, robot_id, room_mapping):
    """Create Turtle for the robot and its gripper."""
    robot_uri = CORP[f"Robot-{robot_id}"]
    graph.add((robot_uri, RDF.type, CORP.Robot))
    graph.add((robot_uri, RDF.type, OWL.NamedIndividual))
    
    # Use the room_mapping for skiros:at relationship
    room_uri = room_mapping.get(robot_data['location'])
    if room_uri:
        graph.add((robot_uri, SKIROS.at, room_uri))
    
    # Add required robot attributes
    graph.add((robot_uri, RDFS.label, Literal("robi:robot")))
    graph.add((robot_uri, SKIROS.BaseFrameId, Literal("", datatype=XSD.string)))
    graph.add((robot_uri, SKIROS.DiscreteReasoner, Literal("AauSpatialReasoner", datatype=XSD.string)))
    graph.add((robot_uri, SKIROS.FrameId, Literal(f"cora:Robot-{robot_id}", datatype=XSD.string)))
    graph.add((robot_uri, SKIROS.PublishTf, Literal(True, datatype=XSD.boolean)))
    graph.add((robot_uri, SKIROS.SkillMgr, Literal("robi_robot", datatype=XSD.string)))
    graph.add((robot_uri, SKIROS.Template, Literal("robi:robot", datatype=XSD.string)))
    graph.add((robot_uri, SKIROS.BatteryPercentage, Literal(100, datatype=XSD.float)))
    graph.add((robot_uri, SKIROS.hasTemplate, ROBI["robot"]))
    
    # Add the gripper
    gripper_uri = RPARTS[f"GripperEffector-{robot_id+1}"]  # Use robot_id+1 for gripper ID
    graph.add((robot_uri, SKIROS.hasA, gripper_uri))
    
    # Add gripper description
    graph.add((gripper_uri, RDF.type, RPARTS.GripperEffector))
    graph.add((gripper_uri, RDF.type, OWL.NamedIndividual))
    graph.add((gripper_uri, RDFS.label, Literal("Robi robot gripper")))
    graph.add((gripper_uri, SKIROS.DiscreteReasoner, Literal("AauSpatialReasoner", datatype=XSD.string)))
    graph.add((gripper_uri, SKIROS.ContainerState, Literal("Empty", datatype=XSD.string)))

    return robot_uri

# Create Turtle for rooms
def create_room_ttl(graph, room_data, room_id, room_mapping):
    """Create Turtle for rooms and map their labels to URIs."""
    room_uri = SKIROS[f"Room-{room_id}"]
    graph.add((room_uri, RDF.type, SKIROS.Room))
    graph.add((room_uri, RDF.type, OWL.NamedIndividual))
    graph.add((room_uri, RDFS.label, Literal(room_data['name'])))
    
    # Store in room_mapping
    room_mapping[room_data['name']] = room_uri
    
    return room_uri

# Create Turtle for locations
def create_location_ttl(graph, location_data, location_id, location_mapping):
    """Create Turtle for locations, handling openable locations."""
    if location_data["category"] == "storage":
        if location_data["name"] == "pantry":
            rdf_type = SKIROS.Pantry
            rdf_name = "Pantry"
        elif location_data["name"] == "fridge":
            rdf_type = SKIROS.Fridge
            rdf_name = "Fridge"
        elif location_data["category"] == "trashcan_large":
            rdf_type = SKIROS.Dumpster
            rdf_name = "Dumpster"
        else:
            rdf_type = SKIROS.OpenableLocation
            rdf_name = "OpenableLocation"
    elif location_data["category"] == "table":
        rdf_type = SKIROS.Table
        rdf_name = "Table"
    elif location_data["category"] == "charger":
        rdf_type = SKIROS.Charger
        rdf_name = "Charger"
    else:
        rdf_type = SKIROS.Location
        rdf_name = "Location"
    location_uri = SKIROS[f"{rdf_name}-{location_id}"]
    graph.add((location_uri, RDF.type, rdf_type))
    graph.add((location_uri, RDF.type, OWL.NamedIndividual))
    graph.add((location_uri, RDFS.label, Literal(location_data['name'])))
    if 'is_open' in location_data:
        graph.add((location_uri, SKIROS.Open, Literal(location_data['is_open'], datatype=XSD.boolean)))
    
    # Store in location_mapping
    location_mapping[location_data['name']] = location_uri
    
    return location_uri

# Create Turtle for doors/hallways (modeled as doors)
def create_door_ttl(graph, door_data, door_id, room_mapping):
    """Create Turtle for doors and hallways."""
    door_label = f"hall_{door_data['room_start']}_{door_data['room_end']}"
    door_uri = SKIROS[f"Door-{door_id}"]
    
    graph.add((door_uri, RDF.type, SKIROS.Door))
    graph.add((door_uri, RDF.type, OWL.NamedIndividual))
    graph.add((door_uri, RDFS.label, Literal(door_label)))

    # Use the room_mapping for skiros:adjacentLocation
    room_start_uri = room_mapping.get(door_data['room_start'])
    room_end_uri = room_mapping.get(door_data['room_end'])
    if room_start_uri and room_end_uri:
        graph.add((door_uri, SKIROS.adjacentLocation, room_start_uri))
        graph.add((door_uri, SKIROS.adjacentLocation, room_end_uri))

    # Add skiros:Open based on door's state
    if 'is_open' in door_data:
        graph.add((door_uri, SKIROS.Open, Literal(door_data['is_open'], datatype=XSD.boolean)))
    
    return door_uri

# Create Turtle for objects
def create_object_ttl(graph, object_data, part_id, location_mapping):
    """Create Turtle for objects and associate them with their locations."""
    part_uri = SKIROS[f"Part-{part_id}"]
    graph.add((part_uri, RDF.type, SKIROS.Part))
    graph.add((part_uri, RDF.type, OWL.NamedIndividual))
    graph.add((part_uri, RDFS.label, Literal(object_data['category'])))
    
    # Use the location_mapping for skiros:contain
    location_uri = location_mapping.get(object_data['parent'])
    if location_uri:
        graph.add((location_uri, SKIROS.contain, part_uri))
    
    return part_uri

# YAML to Turtle conversion
def yaml_to_turtle(yaml_data, world_number: int):
    """Convert YAML data to Turtle format."""
    graph = Graph()
    
    graph.bind("cora", CORP)
    graph.bind("skiros", SKIROS)
    graph.bind("rdfs", RDFS)
    graph.bind("owl", OWL)
    graph.bind("xsd", XSD)
    graph.bind("robi", ROBI)
    graph.bind("rparts", RPARTS)

    id_gen = IDGenerator()
    room_mapping = {}      # Mapping of room labels to room URIs
    location_mapping = {}  # Mapping of location labels to location URIs

    # Create Scene (always Scene-0)
    scene_uri = create_scene_ttl(graph, world_number)
    room_uris = []
    door_uris = []

    # Create rooms
    for room in yaml_data.get('rooms', []):
        room_uri = create_room_ttl(graph, room, id_gen.get_next(), room_mapping)
        room_uris.append(room_uri)

    # Create doors/hallways
    for door in yaml_data.get('hallways', []):
        door_uri = create_door_ttl(graph, door, id_gen.get_next(), room_mapping)
        door_uris.append(door_uri)

    # Create robots
    for robot in yaml_data.get('robots', []):
        create_robot_ttl(graph, robot, id_gen.get_next(), room_mapping)

    # Create locations
    for location in yaml_data.get('locations', []):
        location_uri = create_location_ttl(graph, location, id_gen.get_next(), location_mapping)
        
        # Ensure parent containment
        parent_uri = room_mapping.get(location.get('parent'))
        if parent_uri:
            graph.add((parent_uri, SKIROS.contain, location_uri))

    # Create objects
    for obj in yaml_data.get('objects', []):
        create_object_ttl(graph, obj, id_gen.get_next(), location_mapping)

    # Add skiros:contain for rooms and doors in the scene
    for room_uri in room_uris:
        graph.add((scene_uri, SKIROS.contain, room_uri))
    for door_uri in door_uris:
        graph.add((scene_uri, SKIROS.contain, door_uri))

    return graph.serialize(format='turtle')

# Main function
if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Convert YAML data to Turtle format.")
    # make the input_file and output_file arguments optional
    parser.add_argument('input_file', nargs='?', help="Path to the input YAML file")
    parser.add_argument('output_file', nargs='?', help="Path to the output Turtle file")
    parser.add_argument('world_number', nargs='?', help="World number to convert")
    args = parser.parse_args()

    NUMBER_OF_PROBLEMS = 4
    # If input or output file is not provided, the script converts all 4 problems
    if args.input_file is None:
        # get input files like world1.yaml from a ROS 2 package called delib_ws_worlds
        # get full path of ROS 2 package
        input_files = [os.path.join(get_package_share_directory("delib_ws_worlds"), "worlds", f"world{i}.yaml") for i in range(1, NUMBER_OF_PROBLEMS + 1)]
        output_files = [os.path.join(get_package_share_directory("skiros2_pyrobosim_lib"), "owl", f"p{i}_scene.turtle") for i in range(1, NUMBER_OF_PROBLEMS + 1)]
        world_numbers = [i for i in range(1, NUMBER_OF_PROBLEMS + 1)]
    else:
        input_files = [args.input_file]
        output_files = [args.output_file]
        world_numbers = [args.world_number]

    for input_file, output_file, world_number in zip(input_files, output_files, world_numbers):
        try:
            yaml_data = load_yaml(input_file)
            turtle_output = yaml_to_turtle(yaml_data, world_number)
            with open(output_file, 'w') as f:
                f.write(turtle_output)
            print(f"Turtle data successfully generated for {input_file} and saved to {output_file}")
        except Exception as e:
            print(f"An error occurred: {e}")