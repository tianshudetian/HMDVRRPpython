import sys
venv_path = r'E:\HMDVRRPpython\.venv\Lib\site-packages'
if venv_path in sys.path:
    sys.path.remove(venv_path)
sys.path.insert(0, venv_path)

from instanceGenerate import generate_instance
# from gurobiSolveCSECv3 import gurobiSolver
from LBBD.nh_lbbd_solver import gurobiSolver
from utils import instance_save, is_file_in_folder, read_json_from_file, dict_to_data_instance, parse_string
from utils import instancePlot
instance_name = "M-d2-n20-k2-p2.json"
folder_path = "instances"
if is_file_in_folder(instance_name, folder_path):
    data = read_json_from_file(instance_name, folder_path)
    myInstance = dict_to_data_instance(data)
else:
    info_dict = parse_string(instance_name)
    myInstance = generate_instance(info_dict)
    instance_save(folder_path, myInstance)
# instancePlot(myInstance)
# gurobiSolver(myInstance)
# gurobiSolver(myInstance, feasibility_cut_mode="xyz_only")
gurobiSolver(myInstance, feasibility_cut_mode="exact")
