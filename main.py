from instanceGenerate import generate_instance
# from gurobiSolveC import gurobiSolver
from gurobiSolveCSECv1 import gurobiSolver
from utils import instance_save, is_file_in_folder, read_json_from_file, dict_to_data_instance, parse_string
from utils import instancePlot
instance_name = "M-d3-n6-k1-p2.json"
folder_path = "instances"
if is_file_in_folder(instance_name, folder_path):
    data = read_json_from_file(instance_name, folder_path)
    myInstance = dict_to_data_instance(data)
else:
    info_dict = parse_string(instance_name)
    myInstance = generate_instance(info_dict)
    instance_save(folder_path, myInstance)
# instancePlot(myInstance)
gurobiSolver(myInstance)
