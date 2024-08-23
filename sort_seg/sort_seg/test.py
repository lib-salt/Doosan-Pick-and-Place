import os
from pathlib import Path
print("here", os.path.join(os.path.dirname("Sort-and-Segregate"), "/common2/imp"))

# print(os.path.join(os.path.dirname(__file__), '..'))
print(os.path.abspath(os.path.join(os.path.abspath(__file__), '../')))
# print(os.path.dirname(os.path.dirname(os.path.realpath(__file__))))
# print(os.path.abspath(os.path.join(os.path.abspath(__file__),"../../common2/imp")))
# set_1 = Path(__file__).parent
# print(os.path.abspath(__file__),"../../common2/imp")
# set_2 = os.chdir(set_1)
# print(os.path.abspath(__file__), "../../common2/imp" )