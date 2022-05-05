#!/usr/bin/env python3

"""
What is this code?
  - ox module for ontology-based explanations. 
  You can expect to find here all the implemented methods that are used in the ontology-based explanations algorithm (OX).
  For the time being, only the methods to get the triples from the knowledge based are implemented in this code. We will 
  include the rest of the methods about constructing the textual explanation using the triples. 

What is defined in it?
  - Two main routines of the OX algorithm: 'get_explanation_triples' and 'construct_explanation'. Other methods are also
  defined and used to build the two main ones. 


What information could one find useful when using this code?
  - Note that if a query returns 'false' the generator 'query.solutions()' seems to be empty, the code would work but the 
  explanation would be empty.

  - Triples in this work are lists of six elements, the first trhee are the usual triple elements (subject, property and
  object), the next two denote the time interval in which the triple holds and the last one indicates whether or not the 
  triple's relation should be negative (e.g., 'not has participant')

  - It is good if you write your ontological classes, relationships and individuals withouth spaces and with every word 
  starting character in uppercase (e.g., 'PlanAdapatation', 'hasParticiant'). In order to enhance the readability of the 
  explanations, we split those names using the uppercase characters, and we replace the upper cases in the relatiohnships. 

  - Note that we use query the knowledge base using time intervals, which adds some complexity to the process. When the
  interval is bound (you write a value), the query will only return true if both intervals are the ones you provided. 
  Hence, most of the queries would return false. That is why we have added some logic to our queries to get all the 
  answers that either are included in the bound interval or include the bound interval: 
    (T1>="+str(t_locality[0])+", T2=<"+str(t_locality[1])+"; "+str(t_locality[0])+">=T1, "+str(t_locality[1])+"=<T2)
  For instance, if we want all the collaborations that exist in the interval 15.0-30.0, we will get a collaboration
  that lasted from 20.0 to 25.0 and also others that lasted from 10.0 to 20.0 or from 28.0 to 40.0. In summary, all
  the instances that were true in that interval. 
    -> We could decide to be more strict and only consider instances whose interval is inside of the provided one.

"""

import re
from oxcra_prolog_module import *
from rosprolog_client import PrologException, Prolog


def get_explanation_triples_(client_rosprolog, ontological_classes, t_locality, specificity):
    triples = dict()

    ont_property_inverse_dict = get_ontology_property_and_inverse_dict(client_rosprolog)

    for ontological_class in ontological_classes:
        # extract the instances of the target classes
        class_instances = get_instances_of_target_class(client_rosprolog, ontological_class, t_locality) 

        #for assertion_type, instances_list in class_instances.items():
        for class_instance, instance_time_interval in class_instances.items():
            # initialize the dictionary key
            triples[class_instance] = list()

            if (specificity < 1 or specificity > 3):
                print("Error while executing get_explanation_triples, inccorrect specificity value.")
            else:
                if (specificity >= 1):
                    get_explanation_triples_specificity_one(client_rosprolog, class_instance, instance_time_interval, triples, ont_property_inverse_dict)
                else:
                    pass
                if (specificity >= 2):
                    get_explanation_triples_specificity_two(client_rosprolog, class_instance, instance_time_interval, triples, ont_property_inverse_dict)
                else:
                    pass
                if (specificity == 3):
                    get_explanation_triples_specificity_three(client_rosprolog, class_instance, instance_time_interval, triples, ont_property_inverse_dict)
                else:
                    pass
            
    return triples


def construct_explanation(client_rosprolog, target_instance, triples_in):
    sentence = ''

    ont_property_inverse_dict = get_ontology_property_and_inverse_dict(client_rosprolog)
    
    mod_triples_cast = cast_triples(target_instance, triples_in, ont_property_inverse_dict)
    mod_triples_clus = cluster_triples(mod_triples_cast)
    mod_triples_ord = order_triples(target_instance, mod_triples_clus)
    sentence = group_triples_in_a_sentence(mod_triples_ord) 
    sentence = sentence + '\n'
        
    return sentence





# get explanation triples functions
def get_explanation_triples_specificity_one(client_rosprolog, class_instance, instance_time_interval, triples, ont_property_inverse_dict):
    # note that the variable triples is modified within this function
    for r in range(2):
        ##print(r)
        if (r == 0):
            q_ = "kb_call(triple("+app_ontology_label+":'"+class_instance+"', R, E) during [T1, T2]), "\
                 "not(dif('http://www.w3.org/1999/02/22-rdf-syntax-ns#type', R))."
            assertion_type = "affirmative"
        else: 
            q_ = "kb_call(triple(D, Rd, owl:'NegativePropertyAssertion') during [T1, T2]), "\
                 "kb_call(triple(D, owl:'sourceIndividual', "+app_ontology_label+":'"+class_instance+"') during [T1, T2]), "\
                 "kb_call(triple(D, owl:'assertionProperty', R) during [T1, T2]), kb_call(triple(D, owl:'targetIndividual', E) during [T1, T2]), "\
                 "not(dif('http://www.w3.org/1999/02/22-rdf-syntax-ns#type', R))."
            assertion_type = "negative"

        query = client_rosprolog.query(q_)

        for solution in query.solutions():
            class_ont_uri = app_ontology_label
            tr_ = kb_solution_to_triple(assertion_type, class_ont_uri, class_instance, solution) 
            #print(tr_)
            if (extract_individual_from_triple_element(tr_[3]) == str(instance_time_interval[0]) and extract_individual_from_triple_element(tr_[4]) == str(instance_time_interval[1]) and triples[class_instance]):
                tr_[3] = ''
                tr_[4] = ''
            else:
                pass

            tr_inv_ = invert_triple(tr_, ont_property_inverse_dict)
            if tr_ in triples[class_instance]:
                #print("Triple already in list.")
                pass
            elif tr_inv_ in triples[class_instance]:
                #print("Inverse triple already in list.")
                pass
            else:
                triples[class_instance].append(tr_)

        query.finish()


def get_explanation_triples_specificity_two(client_rosprolog, class_instance, instance_time_interval, triples, ont_property_inverse_dict):
    for r in range(2):
        ##print(r)
        if (r == 0):
            q_ = "kb_call(triple("+app_ontology_label+":'"+class_instance+"', R, E) during [T1, T2]), "\
                 "dif('http://www.w3.org/1999/02/22-rdf-syntax-ns#type', R)."
            assertion_type = "affirmative"
        else: 
            q_ = "kb_call(triple(D, Rd, owl:'NegativePropertyAssertion') during [T1, T2]), "\
                 "kb_call(triple(D, owl:'sourceIndividual', "+app_ontology_label+":'"+class_instance+"') during [T1, T2]), "\
                 "kb_call(triple(D, owl:'assertionProperty', R) during [T1, T2]), kb_call(triple(D, owl:'targetIndividual', E) during [T1, T2]), "\
                 "dif('http://www.w3.org/1999/02/22-rdf-syntax-ns#type', R)."
            assertion_type = "negative"

        query = client_rosprolog.query(q_)

        for solution in query.solutions():
            class_ont_uri = app_ontology_label
            tr_ = kb_solution_to_triple(assertion_type, class_ont_uri, class_instance, solution)
            #print(tr_)
            if (extract_individual_from_triple_element(tr_[3]) == str(instance_time_interval[0]) and extract_individual_from_triple_element(tr_[4]) == str(instance_time_interval[1])):
                tr_[3] = ''
                tr_[4] = ''
            else:
                pass

            tr_inv_ = invert_triple(tr_, ont_property_inverse_dict)
            if tr_ in triples[class_instance]:
                #print("Triple already in list.")
                pass
            elif tr_inv_ in triples[class_instance]:
                #print("Inverse triple already in list.")
                pass
            else:
                triples[class_instance].append(tr_)

        query.finish()


def get_explanation_triples_specificity_three(client_rosprolog, class_instance, instance_time_interval, triples, ont_property_inverse_dict):
    for r in range(4):
        ##print(r)
        if (r == 0):
            q_ = "kb_call(triple("+app_ontology_label+":'"+class_instance+"', Rx, Ex) during [Tx1, Tx2]), "\
                 "dif('http://www.w3.org/1999/02/22-rdf-syntax-ns#type', Rx), "\
                 "kb_call((triple(Ex, R, E) during [T1, T2], (T1>="+str(instance_time_interval[0])+", T1=<"+str(instance_time_interval[1])+"; "\
                 "T2>="+str(instance_time_interval[0])+", T2=<"+str(instance_time_interval[1])+"; "+str(instance_time_interval[0])+">=T1, "+str(instance_time_interval[1])+"=<T2)))."
            assertion_type = "affirmative"
        elif (r == 1):
            q_ = "kb_call(triple("+app_ontology_label+":'"+class_instance+"', Rx, Ex) during [Tx1, Tx2]), "\
                 "dif('http://www.w3.org/1999/02/22-rdf-syntax-ns#type', Rx), " \
                 "kb_call((triple(D, Rd, owl:'NegativePropertyAssertion') during [T1, T2], (T1>="+str(instance_time_interval[0])+", T1=<"+str(instance_time_interval[1])+"; "\
                 "T2>="+str(instance_time_interval[0])+", T2=<"+str(instance_time_interval[1])+"; "+str(instance_time_interval[0])+">=T1, "+str(instance_time_interval[1])+"=<T2))), "\
                 "kb_call(triple(D, owl:'sourceIndividual', Ex) during [T1, T2]), " \
                 "kb_call(triple(D, owl:'assertionProperty', R) during [T1, T2]), kb_call(triple(D, owl:'targetIndividual', E) during [T1, T2])."
            assertion_type = "negative"
        elif (r == 2):
            q_ = "kb_call(triple(Dx, Rdx, owl:'NegativePropertyAssertion') during [Tx1, Tx2]), "\
                 "kb_call(triple(Dx, owl:'sourceIndividual', "+app_ontology_label+":'"+class_instance+"') during [Tx1, Tx2]), " \
                 "kb_call(triple(Dx, owl:'assertionProperty', Rx) during [Tx1, Tx2]), kb_call(triple(Dx, owl:'targetIndividual', Ex) during [Tx1, Tx2]), "\
                 "dif('http://www.w3.org/1999/02/22-rdf-syntax-ns#type', Rx), "\
                 "kb_call((triple(Ex, R, E) during [T1, T2], (T1>="+str(instance_time_interval[0])+", T1=<"+str(instance_time_interval[1])+"; "\
                 "T2>="+str(instance_time_interval[0])+", T2=<"+str(instance_time_interval[1])+"; "+str(instance_time_interval[0])+">=T1, "+str(instance_time_interval[1])+"=<T2)))."
            assertion_type = "affirmative"
        else: 
            q_ = "kb_call(triple(Dx, Rdx, owl:'NegativePropertyAssertion') during [Tx1, Tx2]), "\
                 "kb_call(triple(Dx, owl:'sourceIndividual', "+app_ontology_label+":'"+class_instance+"') during [Tx1, Tx2]), " \
                 "kb_call(triple(Dx, owl:'assertionProperty', Rx) during [Tx1, Tx2]), kb_call(triple(Dx, owl:'targetIndividual', Ex) during [Tx1, Tx2]), "\
                 "dif('http://www.w3.org/1999/02/22-rdf-syntax-ns#type', Rx), "\
                 "kb_call((triple(D, Rd, owl:'NegativePropertyAssertion') during [T1, T2], (T1>="+str(instance_time_interval[0])+", T1=<"+str(instance_time_interval[1])+"; "\
                 "T2>="+str(instance_time_interval[0])+", T2=<"+str(instance_time_interval[1])+"; "+str(instance_time_interval[0])+">=T1, "+str(instance_time_interval[1])+"=<T2))), "\
                 "kb_call(triple(D, owl:'sourceIndividual', Ex) during [T1, T2]), kb_call(triple(D, owl:'assertionProperty', R) during [T1, T2]), "\
                 "kb_call(triple(D, owl:'targetIndividual', E) during [T1, T2])."
            assertion_type = "negative"

        query = client_rosprolog.query(q_)


        for solution in query.solutions():
            class_ = extract_individual_from_kb_answer(solution["Ex"]) #solution["Ex"].split('#')[-1]
            class_ont_uri = owl_uri_to_label_dict[extract_raw_uri_from_kb_answer(solution["Ex"])]
            tr_ = kb_solution_to_triple(assertion_type, class_ont_uri, class_, solution)
            ##print(tr_)
            if (extract_individual_from_triple_element(tr_[3]) == str(instance_time_interval[0]) and extract_individual_from_triple_element(tr_[4]) == str(instance_time_interval[1])):
                tr_[3] = ''
                tr_[4] = ''
            else:
                pass

            tr_inv_ = invert_triple(tr_, ont_property_inverse_dict)
            #for existent_tr in triples[class_instance]:
            if tr_ in triples[class_instance]:
            #if all(item in tr_ for item in existent_tr):
            #if tr_[0] == existent_tr[0] and tr_[1] == existent_tr[1] and tr_[2] == existent_tr[2]:
                #print("Triple already in list.")
                pass
            elif tr_inv_ in triples[class_instance]:
            #elif all(item in tr_inv_ for item in existent_tr):
            #if tr_inv_[0] == existent_tr[0] and tr_inv_[1] == existent_tr[1] and tr_inv_[2] == existent_tr[2]:
                #print("Inverse triple already in list.")
                pass
            else:
                triples[class_instance].append(tr_)

        query.finish()





# construct explanation functions
def cast_triples(target_instance, triples_in, ont_prop_dict):
    # rules have been taken from 'Agregation in natural language generation, by H Dalianis'
    # unify properties using their inverse: inverse those where the main instance is object 
    # and also make sure that we do not use a property and its inverse (we change the ones 
    # that appear later when going through the triples)
    casted_triples = []
    triples = triples_in.copy()


    for triple in triples:
        if target_instance in triple[2]:
            ##print("if instance is object")
            if invert_triple(triple, ont_prop_dict) in casted_triples: # avoid repetitions when there are more than 2 target instances
                ##print("if inverted in casted")
                pass
            else:
                casted_triples.append(invert_triple(triple, ont_prop_dict))
        else:
            ##print("if instance is not object")
            triple_property = extract_individual_from_triple_element(triple[1])
            concatenated_casted_triples = []
            casted_triples_cp = casted_triples.copy()
            [concatenated_casted_triples.extend(el) for el in casted_triples_cp]

            if triple_property in ont_prop_dict and any(ont_prop_dict[triple_property] in s for s in concatenated_casted_triples):
                ##print("if prop in casted")
                if invert_triple(triple, ont_prop_dict) in casted_triples: # avoid repetitions e.g. isWorsePlanThan
                    ##print("if inverted in casted")
                    pass
                else:
                    ##print("if inverted not in casted")
                    casted_triples.append(invert_triple(triple, ont_prop_dict))
            else:
                ##print("if prop not in casted")
                if triple in casted_triples: # avoid repetitions when there are more than 2 main instances
                    pass
                else:
                    casted_triples.append(triple)

    
    return casted_triples


def cluster_triples(triples_in):
    # rules have been taken from 'Agregation in natural language generation, by H Dalianis'
    # -triples about a single instance/entity are clustered together
    clustered_triples_dict = dict()
    triples = triples_in.copy()
    
    triples_cp = triples.copy()
    cont = 0
    while triples_cp:
        indices = find_indices_of_related_triples(triples_cp[0], triples_cp) 
        related_triples = extract_related_triples(triples_cp, indices)
        clustered_triples_dict[cont] = related_triples
        indices_to_remove = indices.copy()
        indices_to_remove.append(0)
        triples_cp = [triples_cp[j] for j in range(0, len(triples_cp)) if j not in indices_to_remove] 
        cont += 1
    
    return clustered_triples_dict


def order_triples(target_instance, triples_dict_in):
    # rules have been taken from 'Agregation in natural language generation, by H Dalianis'
    # -(between sentences) external ordering: much info > less info (except the main instance to explain, it always goes first)
    # -(within a sentence) internal ordering: superclass > attributes
    ordered_triples = dict()
    ordered_triples_ext = dict()
    ordered_triples_int = dict()
    triples_dict = triples_dict_in.copy()
    
    # external ordering
    triples_dict_cp = triples_dict.copy()
    cont = 0
    while (triples_dict_cp):
        key_with_max_info = list(triples_dict_cp.keys())[0]
        for key, value in triples_dict_cp.items():
            if any(target_instance in triple_element for triple_element in value[0]):
                key_with_max_info = key
                break
            else:
                if len(triples_dict_cp[key_with_max_info]) < len(value):
                    key_with_max_info = key
                else:
                    pass
        
        ordered_triples_ext[cont] = triples_dict_cp[key_with_max_info]
        triples_dict_cp.pop(key_with_max_info)
        cont += 1
        
    # internal ordering
    for key, value in ordered_triples_ext.items():
        new_value = list()
        
        for v in value:
            if 'type' in v[1]:
                new_value.insert(0, v)
            else:
                new_value.append(v)
        
        ordered_triples_int[key] = new_value
        
    ordered_triples = ordered_triples_int.copy()
    
    return ordered_triples


def group_triples_in_a_sentence(triples_dict_in): # ont_prop_plural_dict
    # rules have been taken from 'Agregation in natural language generation, by H Dalianis'
    # -subject grouping: all triples with the same subject are grouped (and)
    # -object grouping: objects that share subject and property are grouped
    grouped_triples = dict()
    aux_dict = dict()
    sentence = ''
    triples_dict = triples_dict_in.copy()
    
    # object grouping 
    for key, triples in triples_dict.items():
        # object grouping
        grouped_triples_by_object = group_triples_of_same_object_type(triples) 
        # subject grouping and sentence generation
        sentence = sentence + triples_list_to_text_with_aggregation(grouped_triples_by_object) + '. \n\n'
        
    return sentence





# util functions
def kb_solution_to_triple(assertion_type, class_ont_uri, class_instance, solution):
    # note that the query solution should contain the fields 'R', 'E', 'T1' and 'T2', which will be transformed into triples
    triple = list()

    triple.append(class_ont_uri+":"+class_instance)
    triple.append(owl_uri_to_label_dict[extract_raw_uri_from_kb_answer(solution['R'])] + ':' + extract_individual_from_kb_answer(solution['R']))
    triple.append(owl_uri_to_label_dict[extract_raw_uri_from_kb_answer(solution['E'])] + ':' + extract_individual_from_kb_answer(solution['E']))
    triple.append('start:' + str(solution['T1']))
    triple.append('end:' + str(solution['T2']))
    triple.append(assertion_type) # whether the triple was asserted as affirtmative or negative (e.g., 'it is not a collaboration')
    return triple


def get_instances_of_target_class(client_rosprolog, ontological_class, t_locality):
    class_instances = dict()

    # positive instances of the classes
    query = client_rosprolog.query("kb_call((triple(I, rdf:type, "+domain_ontology_label+":'"+ontological_class+"') during[T1, T2], "\
                                   "(T1>="+str(t_locality[0])+", T1=<"+str(t_locality[1])+"; T2>="+str(t_locality[0])+", T2=<"+str(t_locality[1])+"; "\
                                   +str(t_locality[0])+">=T1, "+str(t_locality[1])+"=<T2))).")

    for solution in query.solutions():
        class_instances[solution['I'].split('#')[-1]] =  [solution['T1'], solution['T2']] # instance without ontology iri
        #print('Found solution. I = %s, T1 = %s, T2 = %s' % (solution['I'], solution['T1'], solution['T2']))
    query.finish()

    # negative instances of the classes
    q_ = "kb_call((triple(D, Rd, owl:'NegativePropertyAssertion') during [T1, T2], (T1>="+str(t_locality[0])+", T1=<"+str(t_locality[1])+"; "\
         "T2>="+str(t_locality[0])+", T2=<"+str(t_locality[1])+"; "+str(t_locality[0])+">=T1, "+str(t_locality[1])+"=<T2))), "\
         "kb_call(triple(D, owl:'sourceIndividual', S) during [T1, T2]), kb_call(triple(D, owl:'assertionProperty', rdf:type) during [T1, T2]), "\
         "kb_call(triple(D, owl:'targetIndividual', "+domain_ontology_label+":'"+ontological_class+"') during [T1, T2])."
    query = client_rosprolog.query(q_)
    for solution in query.solutions():
        class_instances[solution['S'].split('#')[-1]] =  [solution['T1'], solution['T2']] 
    query.finish()
    
    return class_instances


def get_ontology_property_and_inverse_dict(client_rosprolog):
    ont_property_inverse_dict = dict()

    query = client_rosprolog.query("kb_call(triple(S, owl:'inverseOf', O))")

    for solution in query.solutions():
        subj_ = solution['S'].split('#')[-1]
        obj_ = solution['O'].split('#')[-1]

        ont_property_inverse_dict[subj_] = obj_
        ont_property_inverse_dict[obj_] = subj_

    query.finish()

    return ont_property_inverse_dict


def invert_triple(triple_in, ont_property_inverse_dict):
    triple = triple_in.copy()
    tr_ = triple.copy()

    ont_uri = extract_uri_label_from_triple_element(triple[1])
    ont_prop = extract_individual_from_triple_element(triple[1])
    if ont_prop in ont_property_inverse_dict:
        tr_[1] = ont_uri+':'+ont_property_inverse_dict[ont_prop]

        tr_[0] = triple[2]
        tr_[2] = triple[0]
        tr_[3] = triple[3]
        tr_[4] = triple[4]
        tr_[5] = triple[5]
    else:
        pass

    return tr_


def extract_individual_from_triple_element(triple_element_in):
    triple_element = str(triple_element_in)
    interesting_information = triple_element.split(':')[-1]
    
    return interesting_information


def extract_uri_label_from_triple_element(triple_element_in):
    triple_element = str(triple_element_in)
    uri = triple_element.split(':')[0]
    
    return uri

def extract_individual_from_kb_answer(kb_answer_element_in):
    kb_answer_element = str(kb_answer_element_in)
    individual = kb_answer_element.split('#')[-1]
    
    return individual


def extract_raw_uri_from_kb_answer(kb_answer_element_in):
    kb_answer_element = str(kb_answer_element_in)
    individual = kb_answer_element.split('#')[-1]
    uri = kb_answer_element.replace(individual, '')
    
    return uri


def find_indices_of_related_triples(triple_in, triples_in):
    indices = []
    triple = triple_in.copy()
    triples = triples_in.copy()
    
    for i in range (0, len(triples)):
        if triple[0] == triples[i][0]:
            indices.append(i)
    
    return indices


def extract_related_triples(triples_in, indices):
    triples = triples_in.copy()

    related_triples = [triples[i] for i in indices]
    
    return related_triples


def group_triples_of_same_object_type(triples_in): # ont_prop_plural_dict
    new_triples = list()
    triples = triples_in.copy()
    
    triples_cp = triples.copy()
    while triples_cp:
        ##print(triples_cp)
        one_triple = triples_cp.pop(0)
        indices_to_remove = []
        for i in range(0, len(triples_cp)):
            if (one_triple[0] == triples_cp[i][0] and one_triple[1] == triples_cp[i][1] and \
               one_triple[3] == triples_cp[i][3] and one_triple[4] == triples_cp[i][4] and \
               one_triple[5] == triples_cp[i][5]):
                # 
                one_triple[1] = extract_individual_from_triple_element(one_triple[1])
                if type(one_triple[2]) == list:
                    one_triple[2].append(triples_cp[i][2])
                else:
                    new_list = []
                    new_list.append(one_triple[2])
                    new_list.append(triples_cp[i][2])
                    one_triple[2] = new_list.copy()


                indices_to_remove.append(i)
            else:
                continue
        
        new_triples.append(one_triple)
        triples_cp = [triples_cp[j] for j in range(0, len(triples_cp)) if j not in indices_to_remove] 
    
    return new_triples


def triples_list_to_text_with_aggregation(triples_in):
    text = ''
    triples = triples_in.copy()
    
    for triple in triples:
        triple_subject = extract_individual_from_triple_element(triple[0])
        triple_subject = "'"+re.sub(r"(?<=\w)([A-Z])", r" \1", triple_subject)+"'" # adding space between words
        
        triple_relationship = extract_individual_from_triple_element(triple[1])
        if triple_relationship == "type":
            triple_relationship = "isATypeOf"
        else:
            pass
        triple_relationship = re.sub(r"(?<=\w)([A-Z])", r" \1", triple_relationship) # adding space between words
        triple_relationship = triple_relationship.lower() # lowercase
        if triple[5] == "negative":
            triple_relationship = "(not) " + triple_relationship
        else:
            pass

        if type(triple[2]) == list:
            triple_object = ''
            for obj in triple[2]:
                if not triple_object:
                    triple_object = extract_individual_from_triple_element(obj)
                else: 
                    triple_object = triple_object + ' and ' + extract_individual_from_triple_element(obj)
        else:
            triple_object = extract_individual_from_triple_element(triple[2])

        triple_object = "'"+re.sub(r"(?<=\w)([A-Z])", r" \1", triple_object)+"'" # adding space between words

        triple_start = extract_individual_from_triple_element(triple[3])
        triple_end = extract_individual_from_triple_element(triple[4])

        if not text: 
            # empty text
            if (triple_start and triple_end):
                text = triple_subject + ' ' + triple_relationship + ' ' + triple_object + ' ' + 'from ' + triple_start + ' to ' + triple_end
            else:
                text = triple_subject + ' ' + triple_relationship + ' ' + triple_object 
        else:
            # non-empty text
            if (triple_start and triple_end):
                text = text + ' and ' + triple_relationship + ' ' + triple_object + ' ' + 'from ' + triple_start + ' to ' + triple_end
            else:
                text = text + ' and ' + triple_relationship + ' ' + triple_object 

    return text


def count_string_words(string_, separator):
    string_cp = str(string_)

    number_of_words = len(string_cp.split(separator))

    return number_of_words