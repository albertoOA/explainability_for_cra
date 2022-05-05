#!/usr/bin/env python3

"""
What is this code?
  - prolog module for ontology-based explanations for collaborative robotics and adaptation. 
  You can expect to find here definitions that are useful to query a prolog knowledge base that 
  uses the ontology for collaborative robotics and adaptation (OCRA).

What is defined in it?
  - owl_uri_to_label_dict: it is a dictionary mapping the complete owl uris of the ontologies to 
  the tag labels. 

  - app_ontology_label: it is the label used to refer to the application ontology where the instances 
  to explain are formally defined. In our case, it is 'ocra_filling_a_tray'.

"""

owl_uri_to_label_dict = {"http://www.iri.upc.edu/groups/perception/OCRA/ont/ocra.owl#": "ocra", \
                         "http://www.iri.upc.edu/groups/perception/OCRA/ont/ocra_filling_a_tray.owl#": "ocra_filling_a_tray", \
                         "http://www.ontologydesignpatterns.org/ont/dul/DUL.owl#": "dul", \
                         "http://www.w3.org/1999/02/22-rdf-syntax-ns#": "rdf"
}

app_ontology_label = "ocra_filling_a_tray"

domain_ontology_label = "ocra"