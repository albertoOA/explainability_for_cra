#!/usr/bin/env python3

"""
What is this code?
  - ox test for ontology-based explanations. 
  You can expect to find here an example of use of the ontology-based explanations algorithm (OX) in the 
  collaborative robotics and adaptation domain.

"""

import re
import rospy
import roslib
import rospkg
from ox_module import *
from oxcra_prolog_module import *
from rosprolog_client import PrologException, Prolog


if __name__ == '__main__':
    rospy.init_node('ox_test')
    roslib.load_manifest('rosprolog')
    
    # ROS useful variables
    client_rosprolog = Prolog()
    rospack = rospkg.RosPack()

    # settings variables
    t_locality = [0.0, 1000.0]
    specificity = 3 # from 1 to 3
    classes_to_query = ["Collaboration", "PlanAdaptation"]
    explanations_file = rospack.get_path('explainability_for_cra') + "/txt/generated_explanations_with_specificity_" + str(specificity) + ".txt"
    explanations_length_file = rospack.get_path('explainability_for_cra') + "/txt/generated_explanations_length_with_specificity_" + str(specificity) + ".csv"


    triples_dict = get_explanation_triples_(client_rosprolog, classes_to_query, t_locality, specificity)
    #print(triples_dict)

    f = open(explanations_file, "w")
    f_l = open(explanations_length_file, "w")
    all_queried_instances = ""
    all_queried_instances_length = "\n"
    for queried_instance, triples in triples_dict.items():
      introductory_text = "\n\n·····Explanation for: " + queried_instance + "\n"
      explanation = construct_explanation(client_rosprolog, queried_instance, triples)

      f.write(introductory_text)
      f.write(explanation)

      #print(introductory_text) 
      #print(explanation)


      explanation_length = count_string_words(explanation, ' ')
      all_queried_instances_length += str(explanation_length) + ";"
      all_queried_instances += queried_instance + ";"

    f_l.write(all_queried_instances)
    f_l.write(all_queried_instances_length)

    f.close()
    f_l.close()