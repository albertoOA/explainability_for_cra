 : : : INSTANCES INVOLVED IN THE NEEM : : 

kb_project(instance_of(ocra_filling_a_tray:'Human', dul:'PhysicalAgent') during [1.0, 1000.0]).
kb_project(instance_of(ocra_filling_a_tray:'Robot', dul:'PhysicalAgent') during [1.0, 1000.0]).

kb_project(instance_of(ocra_filling_a_tray:'pmLab', ocra:'CollaborationPlace') during [1.0, 1000.0]).

kb_project(instance_of(ocra_filling_a_tray:'PlaceTokensInColumnsByColor', dul:'Plan') during [1.0, 1000.0]).
kb_project(instance_of(ocra_filling_a_tray:'PlaceTokensInAscendingOrder', dul:'Plan') during [1.0, 1000.0]).
kb_project(instance_of(ocra_filling_a_tray:'PlaceTokensWithOddNumbers', dul:'Plan') during [1.0, 1000.0]).
kb_project(instance_of(ocra_filling_a_tray:'PlaceTokenOnCompartment19', dul:'Plan') during [1.0, 1000.0]).
kb_project(instance_of(ocra_filling_a_tray:'StopUntilHumanCommand', dul:'Plan') during [1.0, 1000.0]).
kb_project(instance_of(ocra_filling_a_tray:'PlaceTokenOnCompartment20', dul:'Plan') during [1.0, 1000.0]).
kb_project(instance_of(ocra_filling_a_tray:'PlaceTokenOnTrash', dul:'Plan') during [1.0, 1000.0]).
kb_project(instance_of(ocra_filling_a_tray:'PlaceTokensInDescendingOrder', dul:'Plan') during [1.0, 1000.0]).
kb_project(instance_of(ocra_filling_a_tray:'PlaceTokenOnCompartment15', dul:'Plan') during [1.0, 1000.0]).
kb_project(instance_of(ocra_filling_a_tray:'PlaceTokensInQuadrantsByColor', dul:'Plan') during [1.0, 1000.0]).
kb_project(instance_of(ocra_filling_a_tray:'PlaceTokenOnAuxiliarPile', dul:'Plan') during [1.0, 1000.0]).
kb_project(instance_of(ocra_filling_a_tray:'PlaceTokenOnCompartment18', dul:'Plan') during [1.0, 1000.0]).

kb_project(instance_of(ocra_filling_a_tray:'FullBoardWithTokensInColumnsByColor', dul:'Goal') during [1.0, 1000.0]).
kb_project(instance_of(ocra_filling_a_tray:'FullBoardWithTokensInAscendingOrder', dul:'Goal') during [1.0, 1000.0]).
kb_project(instance_of(ocra_filling_a_tray:'FullBoardWithTokensWithOddNumbers', dul:'Goal') during [1.0, 1000.0]).
kb_project(instance_of(ocra_filling_a_tray:'FullBoardWithTokensInDescendingOrder', dul:'Goal') during [1.0, 1000.0]).
kb_project(instance_of(ocra_filling_a_tray:'FullBoardWithTokensInQuadrantsByColor', dul:'Goal') during [1.0, 1000.0]).



: : : : NON-COLLABORATION _ HumanWriting : : 

kb_project(instance_of(ocra_filling_a_tray:'Event_9', dul:'Event') during [1.0, 41.0]).
kb_project(instance_of(ocra_filling_a_tray:'_:DescriptionRobotX', owl:'NegativePropertyAssertion') during [1.0, 41.0]), kb_project(triple(ocra_filling_a_tray:'_:DescriptionRobotX', owl:'sourceIndividual', ocra_filling_a_tray:'Event_9') during [1.0, 41.0]), kb_project(triple(ocra_filling_a_tray:'_:DescriptionRobotX', owl:'assertionProperty', rdf:'type') during [1.0, 41.0]), kb_project(triple(ocra_filling_a_tray:'_:DescriptionRobotX', owl:'targetIndividual', ocra:'Collaboration') during [1.0, 41.0]).
kb_project(instance_of(ocra_filling_a_tray:'_:DescriptionRobotX0', owl:'NegativePropertyAssertion') during [1.0, 41.0]), kb_project(triple(ocra_filling_a_tray:'_:DescriptionRobotX0', owl:'sourceIndividual', ocra_filling_a_tray:'Human') during [1.0, 41.0]), kb_project(triple(ocra_filling_a_tray:'_:DescriptionRobotX0', owl:'assertionProperty', dul:'isParticipantIn') during [1.0, 41.0]), kb_project(triple(ocra_filling_a_tray:'_:DescriptionRobotX0', owl:'targetIndividual', ocra_filling_a_tray:'Event_9') during [1.0, 41.0]).
kb_project(instance_of(ocra_filling_a_tray:'_:DescriptionRobotX1', owl:'NegativePropertyAssertion') during [1.0, 41.0]), kb_project(triple(ocra_filling_a_tray:'_:DescriptionRobotX1', owl:'sourceIndividual', ocra_filling_a_tray:'Event_9') during [1.0, 41.0]), kb_project(triple(ocra_filling_a_tray:'_:DescriptionRobotX1', owl:'assertionProperty', dul:'hasParticipant') during [1.0, 41.0]), kb_project(triple(ocra_filling_a_tray:'_:DescriptionRobotX1', owl:'targetIndividual', ocra_filling_a_tray:'Human') during [1.0, 41.0]).

kb_project(triple(ocra_filling_a_tray:'Event_9', ocra:executesPlan, ocra_filling_a_tray:'PlaceTokensInColumnsByColor') during [1.0, 41.0]).
kb_project(triple(ocra_filling_a_tray:'PlaceTokensInColumnsByColor', ocra:isPlanExecutedIn, ocra_filling_a_tray:'Event_9') during [1.0, 41.0]).
kb_project(triple(ocra_filling_a_tray:'PlaceTokensInColumnsByColor', dul:hasComponent, ocra_filling_a_tray:'FullBoardWithTokensInColumnsByColor') during [1.0, 41.0]).
kb_project(triple(ocra_filling_a_tray:'FullBoardWithTokensInColumnsByColor', dul:isComponentOf, ocra_filling_a_tray:'PlaceTokensInColumnsByColor') during [1.0, 41.0]).
kb_project(triple(ocra_filling_a_tray:'Event_9', dul:hasParticipant, ocra_filling_a_tray:'Robot') during [1.0, 41.0]).
kb_project(triple(ocra_filling_a_tray:'Robot', dul:isParticipantIn, ocra_filling_a_tray:'Event_9') during [1.0, 41.0]).

kb_project(triple(ocra_filling_a_tray:'PlaceTokensInColumnsByColor', ocra:isPlanOf, ocra_filling_a_tray:'Robot') during [1.0, 41.0]).
kb_project(triple(ocra_filling_a_tray:'Robot', ocra:hasPlan, ocra_filling_a_tray:'PlaceTokensInColumnsByColor') during [1.0, 41.0]).
kb_project(triple(ocra_filling_a_tray:'PlaceTokensInColumnsByColor', ocra:isPlanOf, ocra_filling_a_tray:'Human') during [1.0, 41.0]).
kb_project(triple(ocra_filling_a_tray:'Human', ocra:hasPlan, ocra_filling_a_tray:'PlaceTokensInColumnsByColor') during [1.0, 41.0]).

kb_project(triple(ocra_filling_a_tray:'FullBoardWithTokensInColumnsByColor', ocra:isGoalOf, ocra_filling_a_tray:'Robot') during [1.0, 41.0]).
kb_project(triple(ocra_filling_a_tray:'Robot', ocra:hasGoal, ocra_filling_a_tray:'FullBoardWithTokensInColumnsByColor') during [1.0, 41.0]).
kb_project(triple(ocra_filling_a_tray:'FullBoardWithTokensInColumnsByColor', ocra:isGoalOf, ocra_filling_a_tray:'Human') during [1.0, 41.0]).
kb_project(triple(ocra_filling_a_tray:'Human', ocra:hasGoal, ocra_filling_a_tray:'FullBoardWithTokensInColumnsByColor') during [1.0, 41.0]).



: : : : NON-COLLABORATION _ HumanLeaving : : 

kb_project(instance_of(ocra_filling_a_tray:'Event_15', dul:'Event') during [100.0, 142.0]).
kb_project(instance_of(ocra_filling_a_tray:'_:DescriptionRobotY', owl:'NegativePropertyAssertion') during [100.0, 142.0]), kb_project(triple(ocra_filling_a_tray:'_:DescriptionRobotY', owl:'sourceIndividual', ocra_filling_a_tray:'Event_15') during [100.0, 142.0]), kb_project(triple(ocra_filling_a_tray:'_:DescriptionRobotY', owl:'assertionProperty', rdf:'type') during [100.0, 142.0]), kb_project(triple(ocra_filling_a_tray:'_:DescriptionRobotY', owl:'targetIndividual', ocra:'Collaboration') during [100.0, 142.0]).
kb_project(instance_of(ocra_filling_a_tray:'_:DescriptionRobotY0', owl:'NegativePropertyAssertion') during [100.0, 142.0]), kb_project(triple(ocra_filling_a_tray:'_:DescriptionRobotY0', owl:'sourceIndividual', ocra_filling_a_tray:'Human') during [100.0, 142.0]), kb_project(triple(ocra_filling_a_tray:'_:DescriptionRobotY0', owl:'assertionProperty', dul:'isParticipantIn') during [100.0, 142.0]), kb_project(triple(ocra_filling_a_tray:'_:DescriptionRobotY0', owl:'targetIndividual', ocra_filling_a_tray:'Event_15') during [100.0, 142.0]).
kb_project(instance_of(ocra_filling_a_tray:'_:DescriptionRobotY1', owl:'NegativePropertyAssertion') during [100.0, 142.0]), kb_project(triple(ocra_filling_a_tray:'_:DescriptionRobotY1', owl:'sourceIndividual', ocra_filling_a_tray:'Event_15') during [100.0, 142.0]), kb_project(triple(ocra_filling_a_tray:'_:DescriptionRobotY1', owl:'assertionProperty', dul:'hasParticipant') during [100.0, 142.0]), kb_project(triple(ocra_filling_a_tray:'_:DescriptionRobotY1', owl:'targetIndividual', ocra_filling_a_tray:'Human') during [100.0, 142.0]).

kb_project(triple(ocra_filling_a_tray:'Event_15', ocra:executesPlan, ocra_filling_a_tray:'PlaceTokensInColumnsByColor') during [100.0, 142.0]).
kb_project(triple(ocra_filling_a_tray:'PlaceTokensInColumnsByColor', ocra:isPlanExecutedIn, ocra_filling_a_tray:'Event_15') during [100.0, 142.0]).
kb_project(triple(ocra_filling_a_tray:'PlaceTokensInColumnsByColor', dul:hasComponent, ocra_filling_a_tray:'FullBoardWithTokensInColumnsByColor') during [100.0, 142.0]).
kb_project(triple(ocra_filling_a_tray:'FullBoardWithTokensInColumnsByColor', dul:isComponentOf, ocra_filling_a_tray:'PlaceTokensInColumnsByColor') during [100.0, 142.0]).
kb_project(triple(ocra_filling_a_tray:'Event_15', dul:hasParticipant, ocra_filling_a_tray:'Robot') during [100.0, 142.0]).
kb_project(triple(ocra_filling_a_tray:'Robot', dul:isParticipantIn, ocra_filling_a_tray:'Event_15') during [100.0, 142.0]).

kb_project(triple(ocra_filling_a_tray:'PlaceTokensInColumnsByColor', ocra:isPlanOf, ocra_filling_a_tray:'Robot') during [100.0, 142.0]).
kb_project(triple(ocra_filling_a_tray:'Robot', ocra:hasPlan, ocra_filling_a_tray:'PlaceTokensInColumnsByColor') during [100.0, 142.0]).
kb_project(triple(ocra_filling_a_tray:'PlaceTokensInColumnsByColor', ocra:isPlanOf, ocra_filling_a_tray:'Human') during [100.0, 142.0]).
kb_project(triple(ocra_filling_a_tray:'Human', ocra:hasPlan, ocra_filling_a_tray:'PlaceTokensInColumnsByColor') during [100.0, 142.0]).

kb_project(triple(ocra_filling_a_tray:'FullBoardWithTokensInColumnsByColor', ocra:isGoalOf, ocra_filling_a_tray:'Robot') during [100.0, 142.0]).
kb_project(triple(ocra_filling_a_tray:'Robot', ocra:hasGoal, ocra_filling_a_tray:'FullBoardWithTokensInColumnsByColor') during [100.0, 142.0]).
kb_project(triple(ocra_filling_a_tray:'FullBoardWithTokensInColumnsByColor', ocra:isGoalOf, ocra_filling_a_tray:'Human') during [100.0, 142.0]).
kb_project(triple(ocra_filling_a_tray:'Human', ocra:hasGoal, ocra_filling_a_tray:'FullBoardWithTokensInColumnsByColor') during [100.0, 142.0]).



: : : : NON-COLLABORATION _ HumanHasDifferentPlan : : 

kb_project(instance_of(ocra_filling_a_tray:'Event_27', dul:'Event') during [200.0, 240.0]).
kb_project(instance_of(ocra_filling_a_tray:'_:DescriptionRobotZ', owl:'NegativePropertyAssertion') during [200.0, 240.0]), kb_project(triple(ocra_filling_a_tray:'_:DescriptionRobotZ', owl:'sourceIndividual', ocra_filling_a_tray:'Event_27') during [200.0, 240.0]), kb_project(triple(ocra_filling_a_tray:'_:DescriptionRobotZ', owl:'assertionProperty', rdf:'type') during [200.0, 240.0]), kb_project(triple(ocra_filling_a_tray:'_:DescriptionRobotZ', owl:'targetIndividual', ocra:'Collaboration') during [200.0, 240.0]).
kb_project(instance_of(ocra_filling_a_tray:'_:DescriptionRobotZ0a', owl:'NegativePropertyAssertion') during [200.0, 240.0]), kb_project(triple(ocra_filling_a_tray:'_:DescriptionRobotZ0a', owl:'sourceIndividual', ocra_filling_a_tray:'Human') during [200.0, 240.0]), kb_project(triple(ocra_filling_a_tray:'_:DescriptionRobotZ0a', owl:'assertionProperty', dul:'hasPlan') during [200.0, 240.0]), kb_project(triple(ocra_filling_a_tray:'_:DescriptionRobotZ0a', owl:'targetIndividual', ocra_filling_a_tray:'PlaceTokensInAscendingOrder') during [200.0, 240.0]).
kb_project(instance_of(ocra_filling_a_tray:'_:DescriptionRobotZ0b', owl:'NegativePropertyAssertion') during [200.0, 240.0]), kb_project(triple(ocra_filling_a_tray:'_:DescriptionRobotZ0b', owl:'sourceIndividual', ocra_filling_a_tray:'PlaceTokensInAscendingOrder') during [200.0, 240.0]), kb_project(triple(ocra_filling_a_tray:'_:DescriptionRobotZ0b', owl:'assertionProperty', dul:'isPlanOf') during [200.0, 240.0]), kb_project(triple(ocra_filling_a_tray:'_:DescriptionRobotZ0b', owl:'targetIndividual', ocra_filling_a_tray:'Human') during [200.0, 240.0]).
kb_project(instance_of(ocra_filling_a_tray:'_:DescriptionRobotZ1a', owl:'NegativePropertyAssertion') during [200.0, 240.0]), kb_project(triple(ocra_filling_a_tray:'_:DescriptionRobotZ1a', owl:'sourceIndividual', ocra_filling_a_tray:'Human') during [200.0, 240.0]), kb_project(triple(ocra_filling_a_tray:'_:DescriptionRobotZ1a', owl:'assertionProperty', dul:'hasGoal') during [200.0, 240.0]), kb_project(triple(ocra_filling_a_tray:'_:DescriptionRobotZ1a', owl:'targetIndividual', ocra_filling_a_tray:'FullBoardWithTokensInAscendingOrder') during [200.0, 240.0]).
kb_project(instance_of(ocra_filling_a_tray:'_:DescriptionRobotZ1b', owl:'NegativePropertyAssertion') during [200.0, 240.0]), kb_project(triple(ocra_filling_a_tray:'_:DescriptionRobotZ1b', owl:'sourceIndividual', ocra_filling_a_tray:'FullBoardWithTokensInAscendingOrder') during [200.0, 240.0]), kb_project(triple(ocra_filling_a_tray:'_:DescriptionRobotZ1b', owl:'assertionProperty', dul:'isGoalOf') during [200.0, 240.0]), kb_project(triple(ocra_filling_a_tray:'_:DescriptionRobotZ1b', owl:'targetIndividual', ocra_filling_a_tray:'Human') during [200.0, 240.0]).

kb_project(triple(ocra_filling_a_tray:'Event_27', ocra:executesPlan, ocra_filling_a_tray:'PlaceTokensInColumnsByColor') during [200.0, 240.0]).
kb_project(triple(ocra_filling_a_tray:'PlaceTokensInColumnsByColor', ocra:isPlanExecutedIn, ocra_filling_a_tray:'Event_27') during [200.0, 240.0]).
kb_project(triple(ocra_filling_a_tray:'PlaceTokensInColumnsByColor', dul:hasComponent, ocra_filling_a_tray:'FullBoardWithTokensInColumnsByColor') during [200.0, 240.0]).
kb_project(triple(ocra_filling_a_tray:'FullBoardWithTokensInColumnsByColor', dul:isComponentOf, ocra_filling_a_tray:'PlaceTokensInColumnsByColor') during [200.0, 240.0]).
kb_project(triple(ocra_filling_a_tray:'Event_27', ocra:executesPlan, ocra_filling_a_tray:'PlaceTokensInAscendingOrder') during [200.0, 240.0]).
kb_project(triple(ocra_filling_a_tray:'PlaceTokensInAscendingOrder', ocra:isPlanExecutedIn, ocra_filling_a_tray:'Event_27') during [200.0, 240.0]).
kb_project(triple(ocra_filling_a_tray:'PlaceTokensInAscendingOrder', dul:hasComponent, ocra_filling_a_tray:'FullBoardWithTokensInAscendingOrder') during [200.0, 240.0]).
kb_project(triple(ocra_filling_a_tray:'FullBoardWithTokensInAscendingOrder', dul:isComponentOf, ocra_filling_a_tray:'PlaceTokensInAscendingOrder') during [200.0, 240.0]).
kb_project(triple(ocra_filling_a_tray:'Event_27', dul:hasParticipant, ocra_filling_a_tray:'Robot') during [200.0, 240.0]).
kb_project(triple(ocra_filling_a_tray:'Robot', dul:isParticipantIn, ocra_filling_a_tray:'Event_27') during [200.0, 240.0]).
kb_project(triple(ocra_filling_a_tray:'Event_27', dul:hasParticipant, ocra_filling_a_tray:'Human') during [200.0, 240.0]).
kb_project(triple(ocra_filling_a_tray:'Human', dul:isParticipantIn, ocra_filling_a_tray:'Event_27') during [200.0, 240.0]).

kb_project(triple(ocra_filling_a_tray:'PlaceTokensInAscendingOrder', ocra:isPlanOf, ocra_filling_a_tray:'Robot') during [200.0, 240.0]).
kb_project(triple(ocra_filling_a_tray:'Robot', ocra:hasPlan, ocra_filling_a_tray:'PlaceTokensInAscendingOrder') during [200.0, 240.0]).
kb_project(triple(ocra_filling_a_tray:'PlaceTokensInColumnsByColor', ocra:isPlanOf, ocra_filling_a_tray:'Human') during [200.0, 240.0]).
kb_project(triple(ocra_filling_a_tray:'Human', ocra:hasPlan, ocra_filling_a_tray:'PlaceTokensInColumnsByColor') during [200.0, 240.0]).

kb_project(triple(ocra_filling_a_tray:'FullBoardWithTokensInAscendingOrder', ocra:isGoalOf, ocra_filling_a_tray:'Robot') during [200.0, 240.0]).
kb_project(triple(ocra_filling_a_tray:'Robot', ocra:hasGoal, ocra_filling_a_tray:'FullBoardWithTokensInAscendingOrder') during [200.0, 240.0]).
kb_project(triple(ocra_filling_a_tray:'FullBoardWithTokensInColumnsByColor', ocra:isGoalOf, ocra_filling_a_tray:'Human') during [200.0, 240.0]).
kb_project(triple(ocra_filling_a_tray:'Human', ocra:hasGoal, ocra_filling_a_tray:'FullBoardWithTokensInColumnsByColor') during [200.0, 240.0]).



: : : : COLLABORATION _ OddNumbers : : 

kb_project(instance_of(ocra_filling_a_tray:'Event_28', ocra:'Collaboration') during [300.0, 340.0]).
kb_project(triple(ocra_filling_a_tray:'Event_28', ocra:executesPlan, ocra_filling_a_tray:'PlaceTokensWithOddNumbers') during [300.0, 340.0]).
kb_project(triple(ocra_filling_a_tray:'PlaceTokensWithOddNumbers', ocra:isPlanExecutedIn, ocra_filling_a_tray:'Event_28') during [300.0, 340.0]).
kb_project(triple(ocra_filling_a_tray:'PlaceTokensWithOddNumbers', dul:hasComponent, ocra_filling_a_tray:'FullBoardWithTokensWithOddNumbers') during [300.0, 340.0]).
kb_project(triple(ocra_filling_a_tray:'FullBoardWithTokensWithOddNumbers', dul:isComponentOf, ocra_filling_a_tray:'PlaceTokensWithOddNumbers') during [300.0, 340.0]).
kb_project(triple(ocra_filling_a_tray:'Event_28', dul:hasParticipant, ocra_filling_a_tray:'Robot') during [300.0, 340.0]).
kb_project(triple(ocra_filling_a_tray:'Robot', dul:isParticipantIn, ocra_filling_a_tray:'Event_28') during [300.0, 340.0]).
kb_project(triple(ocra_filling_a_tray:'Event_28', dul:hasParticipant, ocra_filling_a_tray:'Human') during [300.0, 340.0]).
kb_project(triple(ocra_filling_a_tray:'Human', dul:isParticipantIn, ocra_filling_a_tray:'Event_28') during [300.0, 340.0]).

kb_project(triple(ocra_filling_a_tray:'PlaceTokensWithOddNumbers', ocra:isPlanOf, ocra_filling_a_tray:'Robot') during [300.0, 340.0]).
kb_project(triple(ocra_filling_a_tray:'Robot', ocra:hasPlan, ocra_filling_a_tray:'PlaceTokensWithOddNumbers') during [300.0, 340.0]).
kb_project(triple(ocra_filling_a_tray:'PlaceTokensWithOddNumbers', ocra:isPlanOf, ocra_filling_a_tray:'Human') during [300.0, 340.0]).
kb_project(triple(ocra_filling_a_tray:'Human', ocra:hasPlan, ocra_filling_a_tray:'PlaceTokensWithOddNumbers') during [300.0, 340.0]).

kb_project(triple(ocra_filling_a_tray:'FullBoardWithTokensWithOddNumbers', ocra:isGoalOf, ocra_filling_a_tray:'Robot') during [300.0, 340.0]).
kb_project(triple(ocra_filling_a_tray:'Robot', ocra:hasGoal, ocra_filling_a_tray:'FullBoardWithTokensWithOddNumbers') during [300.0, 340.0]).
kb_project(triple(ocra_filling_a_tray:'FullBoardWithTokensWithOddNumbers', ocra:isGoalOf, ocra_filling_a_tray:'Human') during [300.0, 340.0]).
kb_project(triple(ocra_filling_a_tray:'Human', ocra:hasGoal, ocra_filling_a_tray:'FullBoardWithTokensWithOddNumbers') during [300.0, 340.0]).

kb_project(triple(ocra_filling_a_tray:'Event_28', dul:hasLocation, ocra_filling_a_tray:'pmLab') during [300.0, 340.0]).
kb_project(triple(ocra_filling_a_tray:'pmLab', dul:isLocationOf, ocra_filling_a_tray:'Event_28') during [300.0, 340.0]).
kb_project(instance_of(ocra_filling_a_tray:'pmLab', ocra:'CollaborationPlace') during [300.0, 340.0]).

kb_project(triple(ocra_filling_a_tray:'Event_28', dul:isClassifiedBy, ocra_filling_a_tray:'CollaborationClass') during [300.0, 340.0]).
kb_project(triple(ocra_filling_a_tray:'CollaborationClass', dul:classifies, ocra_filling_a_tray:'Event_28') during [300.0, 340.0]).
kb_project(instance_of(ocra_filling_a_tray:'CollaborationClass', ocra:'IndirectlyPhysicalCollaboration') during [300.0, 340.0]).

kb_project(triple(ocra_filling_a_tray:'Event_28', dul:hasQuality, ocra_filling_a_tray:'CurrentRiskOfCollision') during [300.0, 340.0]).
kb_project(triple(ocra_filling_a_tray:'CurrentRiskOfCollision', dul:isQualityOf, ocra_filling_a_tray:'Event_28') during [300.0, 340.0]).
kb_project(instance_of(ocra_filling_a_tray:'CurrentRiskOfCollision', ocra:'CollaborationRisk') during [300.0, 340.0]).
kb_project(triple(ocra_filling_a_tray:'CurrentRiskOfCollision', dul:hasDataValue, ocra_filling_a_tray:'LowRisk') during [300.0, 312.0]).
kb_project(triple(ocra_filling_a_tray:'CurrentRiskOfCollision', dul:hasDataValue, ocra_filling_a_tray:'MediumRisk') during [312.0, 314.0]).
kb_project(triple(ocra_filling_a_tray:'CurrentRiskOfCollision', dul:hasDataValue, ocra_filling_a_tray:'LowRisk') during [314.0, 340.0]).




: : : : COLLABORATION _ AscendingOrder : : 

kb_project(instance_of(ocra_filling_a_tray:'Event_30', ocra:'Collaboration') during [400.0, 440.0]).
kb_project(triple(ocra_filling_a_tray:'Event_30', ocra:executesPlan, ocra_filling_a_tray:'PlaceTokensInAscendingOrder') during [400.0, 440.0]).
kb_project(triple(ocra_filling_a_tray:'PlaceTokensInAscendingOrder', ocra:isPlanExecutedIn, ocra_filling_a_tray:'Event_30') during [400.0, 440.0]).
kb_project(triple(ocra_filling_a_tray:'PlaceTokensInAscendingOrder', dul:hasComponent, ocra_filling_a_tray:'FullBoardWithTokensInAscendingOrder') during [400.0, 440.0]).
kb_project(triple(ocra_filling_a_tray:'FullBoardWithTokensInAscendingOrder', dul:isComponentOf, ocra_filling_a_tray:'PlaceTokensInAscendingOrder') during [400.0, 440.0]).
kb_project(triple(ocra_filling_a_tray:'Event_30', dul:hasParticipant, ocra_filling_a_tray:'Robot') during [400.0, 440.0]).
kb_project(triple(ocra_filling_a_tray:'Robot', dul:isParticipantIn, ocra_filling_a_tray:'Event_30') during [400.0, 440.0]).
kb_project(triple(ocra_filling_a_tray:'Event_30', dul:hasParticipant, ocra_filling_a_tray:'Human') during [400.0, 440.0]).
kb_project(triple(ocra_filling_a_tray:'Human', dul:isParticipantIn, ocra_filling_a_tray:'Event_30') during [400.0, 440.0]).

kb_project(triple(ocra_filling_a_tray:'PlaceTokensInAscendingOrder', ocra:isPlanOf, ocra_filling_a_tray:'Robot') during [400.0, 440.0]).
kb_project(triple(ocra_filling_a_tray:'Robot', ocra:hasPlan, ocra_filling_a_tray:'PlaceTokensInAscendingOrder') during [400.0, 440.0]).
kb_project(triple(ocra_filling_a_tray:'PlaceTokensInAscendingOrder', ocra:isPlanOf, ocra_filling_a_tray:'Human') during [400.0, 440.0]).
kb_project(triple(ocra_filling_a_tray:'Human', ocra:hasPlan, ocra_filling_a_tray:'PlaceTokensInAscendingOrder') during [400.0, 440.0]).

kb_project(triple(ocra_filling_a_tray:'FullBoardWithTokensInAscendingOrder', ocra:isGoalOf, ocra_filling_a_tray:'Robot') during [400.0, 440.0]).
kb_project(triple(ocra_filling_a_tray:'Robot', ocra:hasGoal, ocra_filling_a_tray:'FullBoardWithTokensInAscendingOrder') during [400.0, 440.0]).
kb_project(triple(ocra_filling_a_tray:'FullBoardWithTokensInAscendingOrder', ocra:isGoalOf, ocra_filling_a_tray:'Human') during [400.0, 440.0]).
kb_project(triple(ocra_filling_a_tray:'Human', ocra:hasGoal, ocra_filling_a_tray:'FullBoardWithTokensInAscendingOrder') during [400.0, 440.0]).

kb_project(triple(ocra_filling_a_tray:'Event_30', dul:hasLocation, ocra_filling_a_tray:'pmLab') during [400.0, 440.0]).
kb_project(triple(ocra_filling_a_tray:'pmLab', dul:isLocationOf, ocra_filling_a_tray:'Event_30') during [400.0, 440.0]).
kb_project(instance_of(ocra_filling_a_tray:'pmLab', ocra:'CollaborationPlace') during [400.0, 440.0]).

kb_project(triple(ocra_filling_a_tray:'Event_30', dul:isClassifiedBy, ocra_filling_a_tray:'CollaborationClass') during [400.0, 440.0]).
kb_project(triple(ocra_filling_a_tray:'CollaborationClass', dul:classifies, ocra_filling_a_tray:'Event_30') during [400.0, 440.0]).
kb_project(instance_of(ocra_filling_a_tray:'CollaborationClass', ocra:'IndirectlyPhysicalCollaboration') during [400.0, 440.0]).

kb_project(triple(ocra_filling_a_tray:'Event_30', dul:hasQuality, ocra_filling_a_tray:'CurrentRiskOfCollision') during [400.0, 440.0]).
kb_project(triple(ocra_filling_a_tray:'CurrentRiskOfCollision', dul:isQualityOf, ocra_filling_a_tray:'Event_30') during [400.0, 440.0]).
kb_project(instance_of(ocra_filling_a_tray:'CurrentRiskOfCollision', ocra:'CollaborationRisk') during [400.0, 440.0]).
kb_project(triple(ocra_filling_a_tray:'CurrentRiskOfCollision', dul:hasDataValue, ocra_filling_a_tray:'LowRisk') during [400.0, 432.0]).
kb_project(triple(ocra_filling_a_tray:'CurrentRiskOfCollision', dul:hasDataValue, ocra_filling_a_tray:'MediumRisk') during [432.0, 436.0]).
kb_project(triple(ocra_filling_a_tray:'CurrentRiskOfCollision', dul:hasDataValue, ocra_filling_a_tray:'LowRisk') during [436.0, 440.0]).




: : : : COLLABORATION _ Color : : 

kb_project(instance_of(ocra_filling_a_tray:'Event_33', ocra:'Collaboration') during [500.0, 542.0]).
kb_project(triple(ocra_filling_a_tray:'Event_33', ocra:executesPlan, ocra_filling_a_tray:'PlaceTokensInColumnsByColor') during [500.0, 542.0]).
kb_project(triple(ocra_filling_a_tray:'PlaceTokensInColumnsByColor', ocra:isPlanExecutedIn, ocra_filling_a_tray:'Event_33') during [500.0, 542.0]).
kb_project(triple(ocra_filling_a_tray:'PlaceTokensInColumnsByColor', dul:hasComponent, ocra_filling_a_tray:'FullBoardWithTokensInColumnsByColor') during [500.0, 542.0]).
kb_project(triple(ocra_filling_a_tray:'FullBoardWithTokensInColumnsByColor', dul:isComponentOf, ocra_filling_a_tray:'PlaceTokensInColumnsByColor') during [500.0, 542.0]).
kb_project(triple(ocra_filling_a_tray:'Event_33', dul:hasParticipant, ocra_filling_a_tray:'Robot') during [500.0, 542.0]).
kb_project(triple(ocra_filling_a_tray:'Robot', dul:isParticipantIn, ocra_filling_a_tray:'Event_33') during [500.0, 542.0]).
kb_project(triple(ocra_filling_a_tray:'Event_33', dul:hasParticipant, ocra_filling_a_tray:'Human') during [500.0, 542.0]).
kb_project(triple(ocra_filling_a_tray:'Human', dul:isParticipantIn, ocra_filling_a_tray:'Event_33') during [500.0, 542.0]).

kb_project(triple(ocra_filling_a_tray:'PlaceTokensInColumnsByColor', ocra:isPlanOf, ocra_filling_a_tray:'Robot') during [500.0, 542.0]).
kb_project(triple(ocra_filling_a_tray:'Robot', ocra:hasPlan, ocra_filling_a_tray:'PlaceTokensInColumnsByColor') during [500.0, 542.0]).
kb_project(triple(ocra_filling_a_tray:'PlaceTokensInColumnsByColor', ocra:isPlanOf, ocra_filling_a_tray:'Human') during [500.0, 542.0]).
kb_project(triple(ocra_filling_a_tray:'Human', ocra:hasPlan, ocra_filling_a_tray:'PlaceTokensInColumnsByColor') during [500.0, 542.0]).

kb_project(triple(ocra_filling_a_tray:'FullBoardWithTokensInColumnsByColor', ocra:isGoalOf, ocra_filling_a_tray:'Robot') during [500.0, 542.0]).
kb_project(triple(ocra_filling_a_tray:'Robot', ocra:hasGoal, ocra_filling_a_tray:'FullBoardWithTokensInColumnsByColor') during [500.0, 542.0]).
kb_project(triple(ocra_filling_a_tray:'FullBoardWithTokensInColumnsByColor', ocra:isGoalOf, ocra_filling_a_tray:'Human') during [500.0, 542.0]).
kb_project(triple(ocra_filling_a_tray:'Human', ocra:hasGoal, ocra_filling_a_tray:'FullBoardWithTokensInColumnsByColor') during [500.0, 542.0]).

kb_project(triple(ocra_filling_a_tray:'Event_33', dul:hasLocation, ocra_filling_a_tray:'pmLab') during [500.0, 542.0]).
kb_project(triple(ocra_filling_a_tray:'pmLab', dul:isLocationOf, ocra_filling_a_tray:'Event_33') during [500.0, 542.0]).
kb_project(instance_of(ocra_filling_a_tray:'pmLab', ocra:'CollaborationPlace') during [500.0, 542.0]).

kb_project(triple(ocra_filling_a_tray:'Event_33', dul:isClassifiedBy, ocra_filling_a_tray:'CollaborationClass') during [500.0, 542.0]).
kb_project(triple(ocra_filling_a_tray:'CollaborationClass', dul:classifies, ocra_filling_a_tray:'Event_33') during [500.0, 542.0]).
kb_project(instance_of(ocra_filling_a_tray:'CollaborationClass', ocra:'IndirectlyPhysicalCollaboration') during [500.0, 542.0]).

kb_project(triple(ocra_filling_a_tray:'Event_33', dul:hasQuality, ocra_filling_a_tray:'CurrentRiskOfCollision') during [500.0, 542.0]).
kb_project(triple(ocra_filling_a_tray:'CurrentRiskOfCollision', dul:isQualityOf, ocra_filling_a_tray:'Event_33') during [500.0, 542.0]).
kb_project(instance_of(ocra_filling_a_tray:'CurrentRiskOfCollision', ocra:'CollaborationRisk') during [500.0, 542.0]).
kb_project(triple(ocra_filling_a_tray:'CurrentRiskOfCollision', dul:hasDataValue, ocra_filling_a_tray:'LowRisk') during [500.0, 508.0]).
kb_project(triple(ocra_filling_a_tray:'CurrentRiskOfCollision', dul:hasDataValue, ocra_filling_a_tray:'MediumRisk') during [508.0, 510.0]).
kb_project(triple(ocra_filling_a_tray:'CurrentRiskOfCollision', dul:hasDataValue, ocra_filling_a_tray:'LowRisk') during [510.0, 542.0]).




: : : : ADAPTATION _ HighRisk : : 

kb_project(instance_of(ocra_filling_a_tray:'Event_39', ocra:'PlanAdaptation') during [600.0, 613.0]).
kb_project(triple(ocra_filling_a_tray:'Event_39', dul:hasParticipant, ocra_filling_a_tray:'Robot') during [600.0, 613.0]).
kb_project(triple(ocra_filling_a_tray:'Robot', dul:isParticipantIn, ocra_filling_a_tray:'Event_39') during [600.0, 613.0]).
kb_project(instance_of(ocra_filling_a_tray:'HighCollisionRisk', dul:'Situation') during [607.0, 607.5]).
kb_project(triple(ocra_filling_a_tray:'HighCollisionRisk', dul:isPostconditionOf, ocra_filling_a_tray:'ExecutionOfPlaceTokenOnCompartment19') during [607.0, 613.0]).
kb_project(triple(ocra_filling_a_tray:'ExecutionOfPlaceTokenOnCompartment19', dul:isPreconditionOf, ocra_filling_a_tray:'HighCollisionRisk') during [607.0, 613.0]).
kb_project(triple(ocra_filling_a_tray:'HighCollisionRisk', dul:isPreconditionOf, ocra_filling_a_tray:'ExecutionOfStopUntilHumanCommand') during [607.5, 613.0]).
kb_project(triple(ocra_filling_a_tray:'ExecutionOfStopUntilHumanCommand', dul:isPostconditionOf, ocra_filling_a_tray:'HighCollisionRisk') during [607.5, 613.0]).

kb_project(triple(ocra_filling_a_tray:'ExecutionOfPlaceTokenOnCompartment19', dul:isPartOf, ocra_filling_a_tray:'Event_39') during [600.0, 607.0]).
kb_project(triple(ocra_filling_a_tray:'Event_39', dul:hasPart, ocra_filling_a_tray:'ExecutionOfPlaceTokenOnCompartment19') during [600.0, 607.0]).
kb_project(triple(ocra_filling_a_tray:'ExecutionOfStopUntilHumanCommand', dul:isPartOf, ocra_filling_a_tray:'Event_39') during [607.5, 613.0]).
kb_project(triple(ocra_filling_a_tray:'Event_39', dul:hasPart, ocra_filling_a_tray:'ExecutionOfStopUntilHumanCommand') during [607.5, 613.0]).

kb_project(triple(ocra_filling_a_tray:'ExecutionOfPlaceTokenOnCompartment19', ocra:executesPlan, ocra_filling_a_tray:'PlaceTokenOnCompartment19') during [600.0, 607.0]).
kb_project(triple(ocra_filling_a_tray:'PlaceTokenOnCompartment19', ocra:isPlanExecutedIn, ocra_filling_a_tray:'ExecutionOfPlaceTokenOnCompartment19') during [600.0, 607.0]).
kb_project(triple(ocra_filling_a_tray:'ExecutionOfStopUntilHumanCommand', ocra:executesPlan, ocra_filling_a_tray:'StopUntilHumanCommand') during [607.5, 613.0]).
kb_project(triple(ocra_filling_a_tray:'StopUntilHumanCommand', ocra:isPlanExecutedIn, ocra_filling_a_tray:'ExecutionOfStopUntilHumanCommand') during [607.5, 613.0]).
kb_project(instance_of(ocra_filling_a_tray:'_:DescriptionRobotA1a', owl:'NegativePropertyAssertion') during [607.5, 613.0]), kb_project(triple(ocra_filling_a_tray:'_:DescriptionRobotA1a', owl:'sourceIndividual', ocra_filling_a_tray:'ExecutionOfStopUntilHumanCommand') during [607.5, 613.0]), kb_project(triple(ocra_filling_a_tray:'_:DescriptionRobotA1a', owl:'assertionProperty', ocra:'executesPlan') during [607.5, 613.0]), kb_project(triple(ocra_filling_a_tray:'_:DescriptionRobotA1a', owl:'targetIndividual', ocra_filling_a_tray:'PlaceTokenOnCompartment19') during [607.5, 613.0]).
kb_project(instance_of(ocra_filling_a_tray:'_:DescriptionRobotA1b', owl:'NegativePropertyAssertion') during [607.5, 613.0]), kb_project(triple(ocra_filling_a_tray:'_:DescriptionRobotA1b', owl:'sourceIndividual', ocra_filling_a_tray:'PlaceTokenOnCompartment19') during [607.5, 613.0]), kb_project(triple(ocra_filling_a_tray:'_:DescriptionRobotA1b', owl:'assertionProperty', ocra:'isPlanExecutedIn') during [607.5, 613.0]), kb_project(triple(ocra_filling_a_tray:'_:DescriptionRobotA1b', owl:'targetIndividual', ocra_filling_a_tray:'ExecutionOfStopUntilHumanCommand') during [607.5, 613.0]).

kb_project(instance_of(ocra_filling_a_tray:'ExecutionOfPlaceTokenOnCompartment19', dul:'Event') during [600.0, 607.0]).
kb_project(instance_of(ocra_filling_a_tray:'ExecutionOfStopUntilHumanCommand', dul:'Event') during [607.5, 613.0]).

kb_project(triple(ocra_filling_a_tray:'ExecutionOfPlaceTokenOnCompartment19', dul:hasParticipant, ocra_filling_a_tray:'Robot') during [600.0, 607.0]).
kb_project(triple(ocra_filling_a_tray:'Robot', dul:isParticipantIn, ocra_filling_a_tray:'ExecutionOfPlaceTokenOnCompartment19') during [600.0, 607.0]).
kb_project(triple(ocra_filling_a_tray:'ExecutionOfStopUntilHumanCommand', dul:hasParticipant, ocra_filling_a_tray:'Robot') during [607.5, 613.0]).
kb_project(triple(ocra_filling_a_tray:'Robot', dul:isParticipantIn, ocra_filling_a_tray:'ExecutionOfStopUntilHumanCommand') during [607.5, 613.0]).

kb_project(triple(ocra_filling_a_tray:'PlaceTokenOnCompartment19', ocra:isWorsePlanThan, ocra_filling_a_tray:'StopUntilHumanCommand') during [607.5, 613.0]).
kb_project(triple(ocra_filling_a_tray:'StopUntilHumanCommand', ocra:isBetterPlanThan, ocra_filling_a_tray:'PlaceTokenOnCompartment19') during [607.5, 613.0]).


kb_project(triple(ocra_filling_a_tray:'PlaceTokenOnCompartment19', dul:hasComponent, ocra_filling_a_tray:'FullBoardWithTokensInColumnsByColor') during [600.0, 613.0]).
kb_project(triple(ocra_filling_a_tray:'FullBoardWithTokensInColumnsByColor', dul:isComponentOf, ocra_filling_a_tray:'PlaceTokenOnCompartment19') during [600.0, 613.0]).
kb_project(triple(ocra_filling_a_tray:'StopUntilHumanCommand', dul:hasComponent, ocra_filling_a_tray:'FullBoardWithTokensInColumnsByColor') during [607.5, 613.0]).
kb_project(triple(ocra_filling_a_tray:'FullBoardWithTokensInColumnsByColor', dul:isComponentOf, ocra_filling_a_tray:'StopUntilHumanCommand') during [607.5, 613.0]).

kb_project(triple(ocra_filling_a_tray:'PlaceTokenOnCompartment19', ocra:isPlanOf, ocra_filling_a_tray:'Robot') during [600.0, 607.0]).
kb_project(triple(ocra_filling_a_tray:'Robot', ocra:hasPlan, ocra_filling_a_tray:'PlaceTokenOnCompartment19') during [600.0, 607.0]).
kb_project(triple(ocra_filling_a_tray:'StopUntilHumanCommand', ocra:isPlanOf, ocra_filling_a_tray:'Robot') during [607.5, 613.0]).
kb_project(triple(ocra_filling_a_tray:'Robot', ocra:hasPlan, ocra_filling_a_tray:'StopUntilHumanCommand') during [607.5, 613.0]).

: COLLABORATION BEHIND ADAPTATION
kb_project(instance_of(ocra_filling_a_tray:'Event_38', ocra:'Collaboration') during [600.0, 649.0]).
kb_project(triple(ocra_filling_a_tray:'Event_38', ocra:executesPlan, ocra_filling_a_tray:'PlaceTokensInColumnsByColor') during [600.0, 649.0]).
kb_project(triple(ocra_filling_a_tray:'PlaceTokensInColumnsByColor', ocra:isPlanExecutedIn, ocra_filling_a_tray:'Event_38') during [600.0, 649.0]).
kb_project(triple(ocra_filling_a_tray:'PlaceTokensInColumnsByColor', dul:hasComponent, ocra_filling_a_tray:'FullBoardWithTokensInColumnsByColor') during [600.0, 649.0]).
kb_project(triple(ocra_filling_a_tray:'FullBoardWithTokensInColumnsByColor', dul:isComponentOf, ocra_filling_a_tray:'PlaceTokensInColumnsByColor') during [600.0, 649.0]).
kb_project(triple(ocra_filling_a_tray:'Event_38', dul:hasParticipant, ocra_filling_a_tray:'Robot') during [600.0, 649.0]).
kb_project(triple(ocra_filling_a_tray:'Robot', dul:isParticipantIn, ocra_filling_a_tray:'Event_38') during [600.0, 649.0]).
kb_project(triple(ocra_filling_a_tray:'Event_38', dul:hasParticipant, ocra_filling_a_tray:'Human') during [600.0, 649.0]).
kb_project(triple(ocra_filling_a_tray:'Human', dul:isParticipantIn, ocra_filling_a_tray:'Event_38') during [600.0, 649.0]).

kb_project(triple(ocra_filling_a_tray:'PlaceTokensInColumnsByColor', ocra:isPlanOf, ocra_filling_a_tray:'Robot') during [600.0, 649.0]).
kb_project(triple(ocra_filling_a_tray:'Robot', ocra:hasPlan, ocra_filling_a_tray:'PlaceTokensInColumnsByColor') during [600.0, 649.0]).
kb_project(triple(ocra_filling_a_tray:'PlaceTokensInColumnsByColor', ocra:isPlanOf, ocra_filling_a_tray:'Human') during [600.0, 649.0]).
kb_project(triple(ocra_filling_a_tray:'Human', ocra:hasPlan, ocra_filling_a_tray:'PlaceTokensInColumnsByColor') during [600.0, 649.0]).

kb_project(triple(ocra_filling_a_tray:'FullBoardWithTokensInColumnsByColor', ocra:isGoalOf, ocra_filling_a_tray:'Robot') during [600.0, 649.0]).
kb_project(triple(ocra_filling_a_tray:'Robot', ocra:hasGoal, ocra_filling_a_tray:'FullBoardWithTokensInColumnsByColor') during [600.0, 649.0]).

kb_project(triple(ocra_filling_a_tray:'Event_38', dul:hasLocation, ocra_filling_a_tray:'pmLab') during [600.0, 649.0]).
kb_project(triple(ocra_filling_a_tray:'pmLab', dul:isLocationOf, ocra_filling_a_tray:'Event_38') during [600.0, 649.0]).
kb_project(instance_of(ocra_filling_a_tray:'pmLab', ocra:'CollaborationPlace') during [600.0, 649.0]).




: : : : ADAPTATION _ TrashSmallestIsFull : : 

kb_project(instance_of(ocra_filling_a_tray:'Event_43', ocra:'PlanAdaptation') during [650.0, 665.0]).
kb_project(triple(ocra_filling_a_tray:'Event_43', dul:hasParticipant, ocra_filling_a_tray:'Robot') during [650.0, 665.0]).
kb_project(triple(ocra_filling_a_tray:'Robot', dul:isParticipantIn, ocra_filling_a_tray:'Event_43') during [650.0, 665.0]).
kb_project(instance_of(ocra_filling_a_tray:'TokenNumberIsTooSmall', dul:'Situation') during [658.0, 658.5]).
kb_project(triple(ocra_filling_a_tray:'TokenNumberIsTooSmall', dul:isPostconditionOf, ocra_filling_a_tray:'ExecutionOfPlaceTokenOnCompartment20') during [658.0, 665.0]).
kb_project(triple(ocra_filling_a_tray:'ExecutionOfPlaceTokenOnCompartment20', dul:isPreconditionOf, ocra_filling_a_tray:'TokenNumberIsTooSmall') during [658.0, 665.0]).
kb_project(triple(ocra_filling_a_tray:'TokenNumberIsTooSmall', dul:isPreconditionOf, ocra_filling_a_tray:'ExecutionOfPlaceTokenOnTrash') during [658.5, 665.0]).
kb_project(triple(ocra_filling_a_tray:'ExecutionOfPlaceTokenOnTrash', dul:isPostconditionOf, ocra_filling_a_tray:'TokenNumberIsTooSmall') during [658.5, 665.0]).

kb_project(triple(ocra_filling_a_tray:'ExecutionOfPlaceTokenOnCompartment20', dul:isPartOf, ocra_filling_a_tray:'Event_43') during [650.0, 658.0]).
kb_project(triple(ocra_filling_a_tray:'Event_43', dul:hasPart, ocra_filling_a_tray:'ExecutionOfPlaceTokenOnCompartment20') during [650.0, 658.0]).
kb_project(triple(ocra_filling_a_tray:'ExecutionOfPlaceTokenOnTrash', dul:isPartOf, ocra_filling_a_tray:'Event_43') during [658.5, 665.0]).
kb_project(triple(ocra_filling_a_tray:'Event_43', dul:hasPart, ocra_filling_a_tray:'ExecutionOfPlaceTokenOnTrash') during [658.5, 665.0]).

kb_project(triple(ocra_filling_a_tray:'ExecutionOfPlaceTokenOnCompartment20', ocra:executesPlan, ocra_filling_a_tray:'PlaceTokenOnCompartment20') during [650.0, 658.0]).
kb_project(triple(ocra_filling_a_tray:'PlaceTokenOnCompartment20', ocra:isPlanExecutedIn, ocra_filling_a_tray:'ExecutionOfPlaceTokenOnCompartment20') during [650.0, 658.0]).
kb_project(triple(ocra_filling_a_tray:'ExecutionOfPlaceTokenOnTrash', ocra:executesPlan, ocra_filling_a_tray:'PlaceTokenOnTrash') during [658.5, 665.0]).
kb_project(triple(ocra_filling_a_tray:'PlaceTokenOnTrash', ocra:isPlanExecutedIn, ocra_filling_a_tray:'ExecutionOfPlaceTokenOnTrash') during [658.5, 665.0]).
kb_project(instance_of(ocra_filling_a_tray:'_:DescriptionRobotA2a', owl:'NegativePropertyAssertion') during [658.5, 665.0]), kb_project(triple(ocra_filling_a_tray:'_:DescriptionRobotA2a', owl:'sourceIndividual', ocra_filling_a_tray:'ExecutionOfPlaceTokenOnTrash') during [658.5, 665.0]), kb_project(triple(ocra_filling_a_tray:'_:DescriptionRobotA2a', owl:'assertionProperty', ocra:'executesPlan') during [658.5, 665.0]), kb_project(triple(ocra_filling_a_tray:'_:DescriptionRobotA2a', owl:'targetIndividual', ocra_filling_a_tray:'PlaceTokenOnCompartment20') during [658.5, 665.0]).
kb_project(instance_of(ocra_filling_a_tray:'_:DescriptionRobotA2b', owl:'NegativePropertyAssertion') during [658.5, 665.0]), kb_project(triple(ocra_filling_a_tray:'_:DescriptionRobotA2b', owl:'sourceIndividual', ocra_filling_a_tray:'PlaceTokenOnCompartment20') during [658.5, 665.0]), kb_project(triple(ocra_filling_a_tray:'_:DescriptionRobotA2b', owl:'assertionProperty', ocra:'isPlanExecutedIn') during [658.5, 665.0]), kb_project(triple(ocra_filling_a_tray:'_:DescriptionRobotA2b', owl:'targetIndividual', ocra_filling_a_tray:'ExecutionOfPlaceTokenOnTrash') during [658.5, 665.0]).

kb_project(instance_of(ocra_filling_a_tray:'ExecutionOfPlaceTokenOnCompartment20', dul:'Event') during [650.0, 658.0]).
kb_project(instance_of(ocra_filling_a_tray:'ExecutionOfPlaceTokenOnTrash', dul:'Event') during [658.5, 665.0]).

kb_project(triple(ocra_filling_a_tray:'ExecutionOfPlaceTokenOnCompartment20', dul:hasParticipant, ocra_filling_a_tray:'Robot') during [650.0, 658.0]).
kb_project(triple(ocra_filling_a_tray:'Robot', dul:isParticipantIn, ocra_filling_a_tray:'ExecutionOfPlaceTokenOnCompartment20') during [650.0, 658.0]).
kb_project(triple(ocra_filling_a_tray:'ExecutionOfPlaceTokenOnTrash', dul:hasParticipant, ocra_filling_a_tray:'Robot') during [658.5, 665.0]).
kb_project(triple(ocra_filling_a_tray:'Robot', dul:isParticipantIn, ocra_filling_a_tray:'ExecutionOfPlaceTokenOnTrash') during [658.5, 665.0]).

kb_project(triple(ocra_filling_a_tray:'PlaceTokenOnCompartment20', ocra:isWorsePlanThan, ocra_filling_a_tray:'PlaceTokenOnTrash') during [658.5, 665.0]).
kb_project(triple(ocra_filling_a_tray:'PlaceTokenOnTrash', ocra:isBetterPlanThan, ocra_filling_a_tray:'PlaceTokenOnCompartment20') during [658.5, 665.0]).


kb_project(triple(ocra_filling_a_tray:'PlaceTokenOnCompartment20', dul:hasComponent, ocra_filling_a_tray:'FullBoardWithTokensInDescendingOrder') during [650.0, 665.0]).
kb_project(triple(ocra_filling_a_tray:'FullBoardWithTokensInDescendingOrder', dul:isComponentOf, ocra_filling_a_tray:'PlaceTokenOnCompartment20') during [650.0, 665.0]).
kb_project(triple(ocra_filling_a_tray:'PlaceTokenOnTrash', dul:hasComponent, ocra_filling_a_tray:'FullBoardWithTokensInDescendingOrder') during [658.5, 665.0]).
kb_project(triple(ocra_filling_a_tray:'FullBoardWithTokensInDescendingOrder', dul:isComponentOf, ocra_filling_a_tray:'PlaceTokenOnTrash') during [658.5, 665.0]).

kb_project(triple(ocra_filling_a_tray:'PlaceTokenOnCompartment20', ocra:isPlanOf, ocra_filling_a_tray:'Robot') during [650.0, 658.0]).
kb_project(triple(ocra_filling_a_tray:'Robot', ocra:hasPlan, ocra_filling_a_tray:'PlaceTokenOnCompartment20') during [650.0, 658.0]).
kb_project(triple(ocra_filling_a_tray:'PlaceTokenOnTrash', ocra:isPlanOf, ocra_filling_a_tray:'Robot') during [658.5, 665.0]).
kb_project(triple(ocra_filling_a_tray:'Robot', ocra:hasPlan, ocra_filling_a_tray:'PlaceTokenOnTrash') during [658.5, 665.0]).

: COLLABORATION BEHIND ADAPTATION
kb_project(instance_of(ocra_filling_a_tray:'Event_42', ocra:'Collaboration') during [650.0, 699.0]).
kb_project(triple(ocra_filling_a_tray:'Event_42', ocra:executesPlan, ocra_filling_a_tray:'PlaceTokensInDescendingOrder') during [650.0, 699.0]).
kb_project(triple(ocra_filling_a_tray:'PlaceTokensInDescendingOrder', ocra:isPlanExecutedIn, ocra_filling_a_tray:'Event_42') during [650.0, 699.0]).
kb_project(triple(ocra_filling_a_tray:'PlaceTokensInDescendingOrder', dul:hasComponent, ocra_filling_a_tray:'FullBoardWithTokensInDescendingOrder') during [650.0, 699.0]).
kb_project(triple(ocra_filling_a_tray:'FullBoardWithTokensInDescendingOrder', dul:isComponentOf, ocra_filling_a_tray:'PlaceTokensInDescendingOrder') during [650.0, 699.0]).
kb_project(triple(ocra_filling_a_tray:'Event_42', dul:hasParticipant, ocra_filling_a_tray:'Robot') during [650.0, 699.0]).
kb_project(triple(ocra_filling_a_tray:'Robot', dul:isParticipantIn, ocra_filling_a_tray:'Event_42') during [650.0, 699.0]).
kb_project(triple(ocra_filling_a_tray:'Event_42', dul:hasParticipant, ocra_filling_a_tray:'Human') during [650.0, 699.0]).
kb_project(triple(ocra_filling_a_tray:'Human', dul:isParticipantIn, ocra_filling_a_tray:'Event_42') during [650.0, 699.0]).

kb_project(triple(ocra_filling_a_tray:'PlaceTokensInDescendingOrder', ocra:isPlanOf, ocra_filling_a_tray:'Robot') during [650.0, 699.0]).
kb_project(triple(ocra_filling_a_tray:'Robot', ocra:hasPlan, ocra_filling_a_tray:'PlaceTokensInDescendingOrder') during [650.0, 699.0]).
kb_project(triple(ocra_filling_a_tray:'PlaceTokensInDescendingOrder', ocra:isPlanOf, ocra_filling_a_tray:'Human') during [650.0, 699.0]).
kb_project(triple(ocra_filling_a_tray:'Human', ocra:hasPlan, ocra_filling_a_tray:'PlaceTokensInDescendingOrder') during [650.0, 699.0]).

kb_project(triple(ocra_filling_a_tray:'FullBoardWithTokensInDescendingOrder', ocra:isGoalOf, ocra_filling_a_tray:'Robot') during [650.0, 699.0]).
kb_project(triple(ocra_filling_a_tray:'Robot', ocra:hasGoal, ocra_filling_a_tray:'FullBoardWithTokensInDescendingOrder') during [650.0, 699.0]).

kb_project(triple(ocra_filling_a_tray:'Event_42', dul:hasLocation, ocra_filling_a_tray:'pmLab') during [650.0, 699.0]).
kb_project(triple(ocra_filling_a_tray:'pmLab', dul:isLocationOf, ocra_filling_a_tray:'Event_42') during [650.0, 699.0]).
kb_project(instance_of(ocra_filling_a_tray:'pmLab', ocra:'CollaborationPlace') during [650.0, 699.0]).



: : : : ADAPTATION _ ColorFullCompartment : : 

kb_project(instance_of(ocra_filling_a_tray:'Event_49', ocra:'PlanAdaptation') during [700.0, 715.0]).
kb_project(triple(ocra_filling_a_tray:'Event_49', dul:hasParticipant, ocra_filling_a_tray:'Robot') during [700.0, 715.0]).
kb_project(triple(ocra_filling_a_tray:'Robot', dul:isParticipantIn, ocra_filling_a_tray:'Event_49') during [700.0, 715.0]).
kb_project(instance_of(ocra_filling_a_tray:'TargetCompartmentIsFull', dul:'Situation') during [707.0, 707.5]).
kb_project(triple(ocra_filling_a_tray:'TargetCompartmentIsFull', dul:isPostconditionOf, ocra_filling_a_tray:'ExecutionOfPlaceTokenOnCompartment19') during [707.0, 715.0]).
kb_project(triple(ocra_filling_a_tray:'ExecutionOfPlaceTokenOnCompartment19', dul:isPreconditionOf, ocra_filling_a_tray:'TargetCompartmentIsFull') during [707.0, 715.0]).
kb_project(triple(ocra_filling_a_tray:'TargetCompartmentIsFull', dul:isPreconditionOf, ocra_filling_a_tray:'ExecutionOfPlaceTokenOnCompartment15') during [707.5, 715.0]).
kb_project(triple(ocra_filling_a_tray:'ExecutionOfPlaceTokenOnCompartment15', dul:isPostconditionOf, ocra_filling_a_tray:'TargetCompartmentIsFull') during [707.5, 715.0]).

kb_project(triple(ocra_filling_a_tray:'ExecutionOfPlaceTokenOnCompartment19', dul:isPartOf, ocra_filling_a_tray:'Event_49') during [700.0, 707.0]).
kb_project(triple(ocra_filling_a_tray:'Event_49', dul:hasPart, ocra_filling_a_tray:'ExecutionOfPlaceTokenOnCompartment19') during [700.0, 707.0]).
kb_project(triple(ocra_filling_a_tray:'ExecutionOfPlaceTokenOnCompartment15', dul:isPartOf, ocra_filling_a_tray:'Event_49') during [707.5, 715.0]).
kb_project(triple(ocra_filling_a_tray:'Event_49', dul:hasPart, ocra_filling_a_tray:'ExecutionOfPlaceTokenOnCompartment15') during [707.5, 715.0]).

kb_project(triple(ocra_filling_a_tray:'ExecutionOfPlaceTokenOnCompartment19', ocra:executesPlan, ocra_filling_a_tray:'PlaceTokenOnCompartment19') during [700.0, 707.0]).
kb_project(triple(ocra_filling_a_tray:'PlaceTokenOnCompartment19', ocra:isPlanExecutedIn, ocra_filling_a_tray:'ExecutionOfPlaceTokenOnCompartment19') during [700.0, 707.0]).
kb_project(triple(ocra_filling_a_tray:'ExecutionOfPlaceTokenOnCompartment15', ocra:executesPlan, ocra_filling_a_tray:'PlaceTokenOnCompartment15') during [707.5, 715.0]).
kb_project(triple(ocra_filling_a_tray:'PlaceTokenOnCompartment15', ocra:isPlanExecutedIn, ocra_filling_a_tray:'ExecutionOfPlaceTokenOnCompartment15') during [707.5, 715.0]).
kb_project(instance_of(ocra_filling_a_tray:'_:DescriptionRobotA3a', owl:'NegativePropertyAssertion') during [707.5, 715.0]), kb_project(triple(ocra_filling_a_tray:'_:DescriptionRobotA3a', owl:'sourceIndividual', ocra_filling_a_tray:'ExecutionOfPlaceTokenOnCompartment15') during [707.5, 715.0]), kb_project(triple(ocra_filling_a_tray:'_:DescriptionRobotA3a', owl:'assertionProperty', ocra:'executesPlan') during [707.5, 715.0]), kb_project(triple(ocra_filling_a_tray:'_:DescriptionRobotA3a', owl:'targetIndividual', ocra_filling_a_tray:'PlaceTokenOnCompartment19') during [707.5, 715.0]).
kb_project(instance_of(ocra_filling_a_tray:'_:DescriptionRobotA3b', owl:'NegativePropertyAssertion') during [707.5, 715.0]), kb_project(triple(ocra_filling_a_tray:'_:DescriptionRobotA3b', owl:'sourceIndividual', ocra_filling_a_tray:'PlaceTokenOnCompartment19') during [707.5, 715.0]), kb_project(triple(ocra_filling_a_tray:'_:DescriptionRobotA3b', owl:'assertionProperty', ocra:'isPlanExecutedIn') during [707.5, 715.0]), kb_project(triple(ocra_filling_a_tray:'_:DescriptionRobotA3b', owl:'targetIndividual', ocra_filling_a_tray:'ExecutionOfPlaceTokenOnCompartment15') during [707.5, 715.0]).

kb_project(instance_of(ocra_filling_a_tray:'ExecutionOfPlaceTokenOnCompartment19', dul:'Event') during [700.0, 707.0]).
kb_project(instance_of(ocra_filling_a_tray:'ExecutionOfPlaceTokenOnCompartment15', dul:'Event') during [707.5, 715.0]).

kb_project(triple(ocra_filling_a_tray:'ExecutionOfPlaceTokenOnCompartment19', dul:hasParticipant, ocra_filling_a_tray:'Robot') during [700.0, 707.0]).
kb_project(triple(ocra_filling_a_tray:'Robot', dul:isParticipantIn, ocra_filling_a_tray:'ExecutionOfPlaceTokenOnCompartment19') during [700.0, 707.0]).
kb_project(triple(ocra_filling_a_tray:'ExecutionOfPlaceTokenOnCompartment15', dul:hasParticipant, ocra_filling_a_tray:'Robot') during [707.5, 715.0]).
kb_project(triple(ocra_filling_a_tray:'Robot', dul:isParticipantIn, ocra_filling_a_tray:'ExecutionOfPlaceTokenOnCompartment15') during [707.5, 715.0]).

kb_project(triple(ocra_filling_a_tray:'PlaceTokenOnCompartment19', ocra:isWorsePlanThan, ocra_filling_a_tray:'PlaceTokenOnCompartment15') during [707.5, 715.0]).
kb_project(triple(ocra_filling_a_tray:'PlaceTokenOnCompartment15', ocra:isBetterPlanThan, ocra_filling_a_tray:'PlaceTokenOnCompartment19') during [707.5, 715.0]).


kb_project(triple(ocra_filling_a_tray:'PlaceTokenOnCompartment19', dul:hasComponent, ocra_filling_a_tray:'FullBoardWithTokensInQuadrantsByColor') during [700.0, 715.0]).
kb_project(triple(ocra_filling_a_tray:'FullBoardWithTokensInQuadrantsByColor', dul:isComponentOf, ocra_filling_a_tray:'PlaceTokenOnCompartment19') during [700.0, 715.0]).
kb_project(triple(ocra_filling_a_tray:'PlaceTokenOnCompartment15', dul:hasComponent, ocra_filling_a_tray:'FullBoardWithTokensInQuadrantsByColor') during [707.5, 715.0]).
kb_project(triple(ocra_filling_a_tray:'FullBoardWithTokensInQuadrantsByColor', dul:isComponentOf, ocra_filling_a_tray:'PlaceTokenOnCompartment15') during [707.5, 715.0]).

kb_project(triple(ocra_filling_a_tray:'PlaceTokenOnCompartment19', ocra:isPlanOf, ocra_filling_a_tray:'Robot') during [700.0, 707.0]).
kb_project(triple(ocra_filling_a_tray:'Robot', ocra:hasPlan, ocra_filling_a_tray:'PlaceTokenOnCompartment19') during [700.0, 707.0]).
kb_project(triple(ocra_filling_a_tray:'PlaceTokenOnCompartment15', ocra:isPlanOf, ocra_filling_a_tray:'Robot') during [707.5, 715.0]).
kb_project(triple(ocra_filling_a_tray:'Robot', ocra:hasPlan, ocra_filling_a_tray:'PlaceTokenOnCompartment15') during [707.5, 715.0]).

: COLLABORATION BEHIND ADAPTATION
kb_project(instance_of(ocra_filling_a_tray:'Event_48', ocra:'Collaboration') during [700.0, 749.0]).
kb_project(triple(ocra_filling_a_tray:'Event_48', ocra:executesPlan, ocra_filling_a_tray:'PlaceTokensInQuadrantsByColor') during [700.0, 749.0]).
kb_project(triple(ocra_filling_a_tray:'PlaceTokensInQuadrantsByColor', ocra:isPlanExecutedIn, ocra_filling_a_tray:'Event_48') during [700.0, 749.0]).
kb_project(triple(ocra_filling_a_tray:'PlaceTokensInQuadrantsByColor', dul:hasComponent, ocra_filling_a_tray:'FullBoardWithTokensInQuadrantsByColor') during [700.0, 749.0]).
kb_project(triple(ocra_filling_a_tray:'FullBoardWithTokensInQuadrantsByColor', dul:isComponentOf, ocra_filling_a_tray:'PlaceTokensInQuadrantsByColor') during [700.0, 749.0]).
kb_project(triple(ocra_filling_a_tray:'Event_48', dul:hasParticipant, ocra_filling_a_tray:'Robot') during [700.0, 749.0]).
kb_project(triple(ocra_filling_a_tray:'Robot', dul:isParticipantIn, ocra_filling_a_tray:'Event_48') during [700.0, 749.0]).
kb_project(triple(ocra_filling_a_tray:'Event_48', dul:hasParticipant, ocra_filling_a_tray:'Human') during [700.0, 749.0]).
kb_project(triple(ocra_filling_a_tray:'Human', dul:isParticipantIn, ocra_filling_a_tray:'Event_48') during [700.0, 749.0]).

kb_project(triple(ocra_filling_a_tray:'PlaceTokensInQuadrantsByColor', ocra:isPlanOf, ocra_filling_a_tray:'Robot') during [700.0, 749.0]).
kb_project(triple(ocra_filling_a_tray:'Robot', ocra:hasPlan, ocra_filling_a_tray:'PlaceTokensInQuadrantsByColor') during [700.0, 749.0]).
kb_project(triple(ocra_filling_a_tray:'PlaceTokensInQuadrantsByColor', ocra:isPlanOf, ocra_filling_a_tray:'Human') during [700.0, 749.0]).
kb_project(triple(ocra_filling_a_tray:'Human', ocra:hasPlan, ocra_filling_a_tray:'PlaceTokensInQuadrantsByColor') during [700.0, 749.0]).

kb_project(triple(ocra_filling_a_tray:'FullBoardWithTokensInQuadrantsByColor', ocra:isGoalOf, ocra_filling_a_tray:'Robot') during [700.0, 749.0]).
kb_project(triple(ocra_filling_a_tray:'Robot', ocra:hasGoal, ocra_filling_a_tray:'FullBoardWithTokensInQuadrantsByColor') during [700.0, 749.0]).

kb_project(triple(ocra_filling_a_tray:'Event_48', dul:hasLocation, ocra_filling_a_tray:'pmLab') during [700.0, 749.0]).
kb_project(triple(ocra_filling_a_tray:'pmLab', dul:isLocationOf, ocra_filling_a_tray:'Event_48') during [700.0, 749.0]).
kb_project(instance_of(ocra_filling_a_tray:'pmLab', ocra:'CollaborationPlace') during [700.0, 749.0]).



: : : : ADAPTATION _ ColorFullCompartment : : 

kb_project(instance_of(ocra_filling_a_tray:'Event_51', ocra:'PlanAdaptation') during [750.0, 763.5]).
kb_project(triple(ocra_filling_a_tray:'Event_51', dul:hasParticipant, ocra_filling_a_tray:'Robot') during [750.0, 763.5]).
kb_project(triple(ocra_filling_a_tray:'Robot', dul:isParticipantIn, ocra_filling_a_tray:'Event_51') during [750.0, 763.5]).
kb_project(instance_of(ocra_filling_a_tray:'TargetColorIsFullWithWrongToken', dul:'Situation') during [757.0, 757.5]).
kb_project(triple(ocra_filling_a_tray:'TargetColorIsFullWithWrongToken', dul:isPostconditionOf, ocra_filling_a_tray:'ExecutionOfPlaceTokenOnCompartment19') during [757.0, 763.5]).
kb_project(triple(ocra_filling_a_tray:'ExecutionOfPlaceTokenOnCompartment19', dul:isPreconditionOf, ocra_filling_a_tray:'TargetColorIsFullWithWrongToken') during [757.0, 763.5]).
kb_project(triple(ocra_filling_a_tray:'TargetColorIsFullWithWrongToken', dul:isPreconditionOf, ocra_filling_a_tray:'ExecutionOfPlaceTokenOnAuxiliarPile') during [757.5, 763.5]).
kb_project(triple(ocra_filling_a_tray:'ExecutionOfPlaceTokenOnAuxiliarPile', dul:isPostconditionOf, ocra_filling_a_tray:'TargetColorIsFullWithWrongToken') during [757.5, 763.5]).

kb_project(triple(ocra_filling_a_tray:'ExecutionOfPlaceTokenOnCompartment19', dul:isPartOf, ocra_filling_a_tray:'Event_51') during [750.0, 757.0]).
kb_project(triple(ocra_filling_a_tray:'Event_51', dul:hasPart, ocra_filling_a_tray:'ExecutionOfPlaceTokenOnCompartment19') during [750.0, 757.0]).
kb_project(triple(ocra_filling_a_tray:'ExecutionOfPlaceTokenOnAuxiliarPile', dul:isPartOf, ocra_filling_a_tray:'Event_51') during [757.5, 763.5]).
kb_project(triple(ocra_filling_a_tray:'Event_51', dul:hasPart, ocra_filling_a_tray:'ExecutionOfPlaceTokenOnAuxiliarPile') during [757.5, 763.5]).

kb_project(triple(ocra_filling_a_tray:'ExecutionOfPlaceTokenOnCompartment19', ocra:executesPlan, ocra_filling_a_tray:'PlaceTokenOnCompartment19') during [750.0, 757.0]).
kb_project(triple(ocra_filling_a_tray:'PlaceTokenOnCompartment19', ocra:isPlanExecutedIn, ocra_filling_a_tray:'ExecutionOfPlaceTokenOnCompartment19') during [750.0, 757.0]).
kb_project(triple(ocra_filling_a_tray:'ExecutionOfPlaceTokenOnAuxiliarPile', ocra:executesPlan, ocra_filling_a_tray:'PlaceTokenOnAuxiliarPile') during [757.5, 763.5]).
kb_project(triple(ocra_filling_a_tray:'PlaceTokenOnAuxiliarPile', ocra:isPlanExecutedIn, ocra_filling_a_tray:'ExecutionOfPlaceTokenOnAuxiliarPile') during [757.5, 763.5]).
kb_project(instance_of(ocra_filling_a_tray:'_:DescriptionRobotA4a', owl:'NegativePropertyAssertion') during [757.5, 763.5]), kb_project(triple(ocra_filling_a_tray:'_:DescriptionRobotA4a', owl:'sourceIndividual', ocra_filling_a_tray:'ExecutionOfPlaceTokenOnAuxiliarPile') during [757.5, 763.5]), kb_project(triple(ocra_filling_a_tray:'_:DescriptionRobotA4a', owl:'assertionProperty', ocra:'executesPlan') during [757.5, 763.5]), kb_project(triple(ocra_filling_a_tray:'_:DescriptionRobotA4a', owl:'targetIndividual', ocra_filling_a_tray:'PlaceTokenOnCompartment19') during [757.5, 763.5]).
kb_project(instance_of(ocra_filling_a_tray:'_:DescriptionRobotA4b', owl:'NegativePropertyAssertion') during [757.5, 763.5]), kb_project(triple(ocra_filling_a_tray:'_:DescriptionRobotA4b', owl:'sourceIndividual', ocra_filling_a_tray:'PlaceTokenOnCompartment19') during [757.5, 763.5]), kb_project(triple(ocra_filling_a_tray:'_:DescriptionRobotA4b', owl:'assertionProperty', ocra:'isPlanExecutedIn') during [757.5, 763.5]), kb_project(triple(ocra_filling_a_tray:'_:DescriptionRobotA4b', owl:'targetIndividual', ocra_filling_a_tray:'ExecutionOfPlaceTokenOnAuxiliarPile') during [757.5, 763.5]).

kb_project(instance_of(ocra_filling_a_tray:'ExecutionOfPlaceTokenOnCompartment19', dul:'Event') during [750.0, 757.0]).
kb_project(instance_of(ocra_filling_a_tray:'ExecutionOfPlaceTokenOnAuxiliarPile', dul:'Event') during [757.5, 763.5]).

kb_project(triple(ocra_filling_a_tray:'ExecutionOfPlaceTokenOnCompartment19', dul:hasParticipant, ocra_filling_a_tray:'Robot') during [750.0, 757.0]).
kb_project(triple(ocra_filling_a_tray:'Robot', dul:isParticipantIn, ocra_filling_a_tray:'ExecutionOfPlaceTokenOnCompartment19') during [750.0, 757.0]).
kb_project(triple(ocra_filling_a_tray:'ExecutionOfPlaceTokenOnAuxiliarPile', dul:hasParticipant, ocra_filling_a_tray:'Robot') during [757.5, 763.5]).
kb_project(triple(ocra_filling_a_tray:'Robot', dul:isParticipantIn, ocra_filling_a_tray:'ExecutionOfPlaceTokenOnAuxiliarPile') during [757.5, 763.5]).

kb_project(triple(ocra_filling_a_tray:'PlaceTokenOnCompartment19', ocra:isWorsePlanThan, ocra_filling_a_tray:'PlaceTokenOnAuxiliarPile') during [757.5, 763.5]).
kb_project(triple(ocra_filling_a_tray:'PlaceTokenOnAuxiliarPile', ocra:isBetterPlanThan, ocra_filling_a_tray:'PlaceTokenOnCompartment19') during [757.5, 763.5]).


kb_project(triple(ocra_filling_a_tray:'PlaceTokenOnCompartment19', dul:hasComponent, ocra_filling_a_tray:'FullBoardWithTokensInQuadrantsByColor') during [750.0, 763.5]).
kb_project(triple(ocra_filling_a_tray:'FullBoardWithTokensInQuadrantsByColor', dul:isComponentOf, ocra_filling_a_tray:'PlaceTokenOnCompartment19') during [750.0, 763.5]).
kb_project(triple(ocra_filling_a_tray:'PlaceTokenOnAuxiliarPile', dul:hasComponent, ocra_filling_a_tray:'FullBoardWithTokensInQuadrantsByColor') during [757.5, 763.5]).
kb_project(triple(ocra_filling_a_tray:'FullBoardWithTokensInQuadrantsByColor', dul:isComponentOf, ocra_filling_a_tray:'PlaceTokenOnAuxiliarPile') during [757.5, 763.5]).

kb_project(triple(ocra_filling_a_tray:'PlaceTokenOnCompartment19', ocra:isPlanOf, ocra_filling_a_tray:'Robot') during [750.0, 757.0]).
kb_project(triple(ocra_filling_a_tray:'Robot', ocra:hasPlan, ocra_filling_a_tray:'PlaceTokenOnCompartment19') during [750.0, 757.0]).
kb_project(triple(ocra_filling_a_tray:'PlaceTokenOnAuxiliarPile', ocra:isPlanOf, ocra_filling_a_tray:'Robot') during [757.5, 763.5]).
kb_project(triple(ocra_filling_a_tray:'Robot', ocra:hasPlan, ocra_filling_a_tray:'PlaceTokenOnAuxiliarPile') during [757.5, 763.5]).

: COLLABORATION BEHIND ADAPTATION
kb_project(instance_of(ocra_filling_a_tray:'Event_50', ocra:'Collaboration') during [750.0, 799.0]).
kb_project(triple(ocra_filling_a_tray:'Event_50', ocra:executesPlan, ocra_filling_a_tray:'PlaceTokensInQuadrantsByColor') during [750.0, 799.0]).
kb_project(triple(ocra_filling_a_tray:'PlaceTokensInQuadrantsByColor', ocra:isPlanExecutedIn, ocra_filling_a_tray:'Event_50') during [750.0, 799.0]).
kb_project(triple(ocra_filling_a_tray:'PlaceTokensInQuadrantsByColor', dul:hasComponent, ocra_filling_a_tray:'FullBoardWithTokensInQuadrantsByColor') during [750.0, 799.0]).
kb_project(triple(ocra_filling_a_tray:'FullBoardWithTokensInQuadrantsByColor', dul:isComponentOf, ocra_filling_a_tray:'PlaceTokensInQuadrantsByColor') during [750.0, 799.0]).
kb_project(triple(ocra_filling_a_tray:'Event_50', dul:hasParticipant, ocra_filling_a_tray:'Robot') during [750.0, 799.0]).
kb_project(triple(ocra_filling_a_tray:'Robot', dul:isParticipantIn, ocra_filling_a_tray:'Event_50') during [750.0, 799.0]).
kb_project(triple(ocra_filling_a_tray:'Event_50', dul:hasParticipant, ocra_filling_a_tray:'Human') during [750.0, 799.0]).
kb_project(triple(ocra_filling_a_tray:'Human', dul:isParticipantIn, ocra_filling_a_tray:'Event_50') during [750.0, 799.0]).

kb_project(triple(ocra_filling_a_tray:'PlaceTokensInQuadrantsByColor', ocra:isPlanOf, ocra_filling_a_tray:'Robot') during [750.0, 799.0]).
kb_project(triple(ocra_filling_a_tray:'Robot', ocra:hasPlan, ocra_filling_a_tray:'PlaceTokensInQuadrantsByColor') during [750.0, 799.0]).
kb_project(triple(ocra_filling_a_tray:'PlaceTokensInQuadrantsByColor', ocra:isPlanOf, ocra_filling_a_tray:'Human') during [750.0, 799.0]).
kb_project(triple(ocra_filling_a_tray:'Human', ocra:hasPlan, ocra_filling_a_tray:'PlaceTokensInQuadrantsByColor') during [750.0, 799.0]).

kb_project(triple(ocra_filling_a_tray:'FullBoardWithTokensInQuadrantsByColor', ocra:isGoalOf, ocra_filling_a_tray:'Robot') during [750.0, 799.0]).
kb_project(triple(ocra_filling_a_tray:'Robot', ocra:hasGoal, ocra_filling_a_tray:'FullBoardWithTokensInQuadrantsByColor') during [750.0, 799.0]).

kb_project(triple(ocra_filling_a_tray:'Event_50', dul:hasLocation, ocra_filling_a_tray:'pmLab') during [750.0, 799.0]).
kb_project(triple(ocra_filling_a_tray:'pmLab', dul:isLocationOf, ocra_filling_a_tray:'Event_50') during [750.0, 799.0]).
kb_project(instance_of(ocra_filling_a_tray:'pmLab', ocra:'CollaborationPlace') during [750.0, 799.0]).



: : : : ADAPTATION _ TrashHighestIsFull : : 

kb_project(instance_of(ocra_filling_a_tray:'Event_59', ocra:'PlanAdaptation') during [800.0, 820.0]).
kb_project(triple(ocra_filling_a_tray:'Event_59', dul:hasParticipant, ocra_filling_a_tray:'Robot') during [800.0, 820.0]).
kb_project(triple(ocra_filling_a_tray:'Robot', dul:isParticipantIn, ocra_filling_a_tray:'Event_59') during [800.0, 820.0]).
kb_project(instance_of(ocra_filling_a_tray:'TokenNumberIsTooHigh', dul:'Situation') during [812.0, 812.5]).
kb_project(triple(ocra_filling_a_tray:'TokenNumberIsTooHigh', dul:isPostconditionOf, ocra_filling_a_tray:'ExecutionOfPlaceTokenOnCompartment20') during [812.0, 820.0]).
kb_project(triple(ocra_filling_a_tray:'ExecutionOfPlaceTokenOnCompartment20', dul:isPreconditionOf, ocra_filling_a_tray:'TokenNumberIsTooHigh') during [812.0, 820.0]).
kb_project(triple(ocra_filling_a_tray:'TokenNumberIsTooHigh', dul:isPreconditionOf, ocra_filling_a_tray:'ExecutionOfPlaceTokenOnTrash') during [812.5, 820.0]).
kb_project(triple(ocra_filling_a_tray:'ExecutionOfPlaceTokenOnTrash', dul:isPostconditionOf, ocra_filling_a_tray:'TokenNumberIsTooHigh') during [812.5, 820.0]).

kb_project(triple(ocra_filling_a_tray:'ExecutionOfPlaceTokenOnCompartment20', dul:isPartOf, ocra_filling_a_tray:'Event_59') during [800.0, 812.0]).
kb_project(triple(ocra_filling_a_tray:'Event_59', dul:hasPart, ocra_filling_a_tray:'ExecutionOfPlaceTokenOnCompartment20') during [800.0, 812.0]).
kb_project(triple(ocra_filling_a_tray:'ExecutionOfPlaceTokenOnTrash', dul:isPartOf, ocra_filling_a_tray:'Event_59') during [812.5, 820.0]).
kb_project(triple(ocra_filling_a_tray:'Event_59', dul:hasPart, ocra_filling_a_tray:'ExecutionOfPlaceTokenOnTrash') during [812.5, 820.0]).

kb_project(triple(ocra_filling_a_tray:'ExecutionOfPlaceTokenOnCompartment20', ocra:executesPlan, ocra_filling_a_tray:'PlaceTokenOnCompartment20') during [800.0, 812.0]).
kb_project(triple(ocra_filling_a_tray:'PlaceTokenOnCompartment20', ocra:isPlanExecutedIn, ocra_filling_a_tray:'ExecutionOfPlaceTokenOnCompartment20') during [800.0, 812.0]).
kb_project(triple(ocra_filling_a_tray:'ExecutionOfPlaceTokenOnTrash', ocra:executesPlan, ocra_filling_a_tray:'PlaceTokenOnTrash') during [812.5, 820.0]).
kb_project(triple(ocra_filling_a_tray:'PlaceTokenOnTrash', ocra:isPlanExecutedIn, ocra_filling_a_tray:'ExecutionOfPlaceTokenOnTrash') during [812.5, 820.0]).
kb_project(instance_of(ocra_filling_a_tray:'_:DescriptionRobotA5a', owl:'NegativePropertyAssertion') during [812.5, 820.0]), kb_project(triple(ocra_filling_a_tray:'_:DescriptionRobotA5a', owl:'sourceIndividual', ocra_filling_a_tray:'ExecutionOfPlaceTokenOnTrash') during [812.5, 820.0]), kb_project(triple(ocra_filling_a_tray:'_:DescriptionRobotA5a', owl:'assertionProperty', ocra:'executesPlan') during [812.5, 820.0]), kb_project(triple(ocra_filling_a_tray:'_:DescriptionRobotA5a', owl:'targetIndividual', ocra_filling_a_tray:'PlaceTokenOnCompartment20') during [812.5, 820.0]).
kb_project(instance_of(ocra_filling_a_tray:'_:DescriptionRobotA5b', owl:'NegativePropertyAssertion') during [812.5, 820.0]), kb_project(triple(ocra_filling_a_tray:'_:DescriptionRobotA5b', owl:'sourceIndividual', ocra_filling_a_tray:'PlaceTokenOnCompartment20') during [812.5, 820.0]), kb_project(triple(ocra_filling_a_tray:'_:DescriptionRobotA5b', owl:'assertionProperty', ocra:'isPlanExecutedIn') during [812.5, 820.0]), kb_project(triple(ocra_filling_a_tray:'_:DescriptionRobotA5b', owl:'targetIndividual', ocra_filling_a_tray:'ExecutionOfPlaceTokenOnTrash') during [812.5, 820.0]).

kb_project(instance_of(ocra_filling_a_tray:'ExecutionOfPlaceTokenOnCompartment20', dul:'Event') during [800.0, 812.0]).
kb_project(instance_of(ocra_filling_a_tray:'ExecutionOfPlaceTokenOnTrash', dul:'Event') during [812.5, 820.0]).

kb_project(triple(ocra_filling_a_tray:'ExecutionOfPlaceTokenOnCompartment20', dul:hasParticipant, ocra_filling_a_tray:'Robot') during [800.0, 812.0]).
kb_project(triple(ocra_filling_a_tray:'Robot', dul:isParticipantIn, ocra_filling_a_tray:'ExecutionOfPlaceTokenOnCompartment20') during [800.0, 812.0]).
kb_project(triple(ocra_filling_a_tray:'ExecutionOfPlaceTokenOnTrash', dul:hasParticipant, ocra_filling_a_tray:'Robot') during [812.5, 820.0]).
kb_project(triple(ocra_filling_a_tray:'Robot', dul:isParticipantIn, ocra_filling_a_tray:'ExecutionOfPlaceTokenOnTrash') during [812.5, 820.0]).

kb_project(triple(ocra_filling_a_tray:'PlaceTokenOnCompartment20', ocra:isWorsePlanThan, ocra_filling_a_tray:'PlaceTokenOnTrash') during [812.5, 820.0]).
kb_project(triple(ocra_filling_a_tray:'PlaceTokenOnTrash', ocra:isBetterPlanThan, ocra_filling_a_tray:'PlaceTokenOnCompartment20') during [812.5, 820.0]).

kb_project(triple(ocra_filling_a_tray:'PlaceTokenOnCompartment20', dul:hasComponent, ocra_filling_a_tray:'FullBoardWithTokensInDescendingOrder') during [800.0, 820.0]).
kb_project(triple(ocra_filling_a_tray:'FullBoardWithTokensInDescendingOrder', dul:isComponentOf, ocra_filling_a_tray:'PlaceTokenOnCompartment20') during [800.0, 820.0]).
kb_project(triple(ocra_filling_a_tray:'PlaceTokenOnTrash', dul:hasComponent, ocra_filling_a_tray:'FullBoardWithTokensInDescendingOrder') during [812.5, 820.0]).
kb_project(triple(ocra_filling_a_tray:'FullBoardWithTokensInDescendingOrder', dul:isComponentOf, ocra_filling_a_tray:'PlaceTokenOnTrash') during [812.5, 820.0]).

kb_project(triple(ocra_filling_a_tray:'PlaceTokenOnCompartment20', ocra:isPlanOf, ocra_filling_a_tray:'Robot') during [800.0, 812.0]).
kb_project(triple(ocra_filling_a_tray:'Robot', ocra:hasPlan, ocra_filling_a_tray:'PlaceTokenOnCompartment20') during [800.0, 812.0]).
kb_project(triple(ocra_filling_a_tray:'PlaceTokenOnTrash', ocra:isPlanOf, ocra_filling_a_tray:'Robot') during [812.5, 820.0]).
kb_project(triple(ocra_filling_a_tray:'Robot', ocra:hasPlan, ocra_filling_a_tray:'PlaceTokenOnTrash') during [812.5, 820.0]).

: COLLABORATION BEHIND ADAPTATION
kb_project(instance_of(ocra_filling_a_tray:'Event_58', ocra:'Collaboration') during [800.0, 849.0]).
kb_project(triple(ocra_filling_a_tray:'Event_58', ocra:executesPlan, ocra_filling_a_tray:'PlaceTokensInDescendingOrder') during [800.0, 849.0]).
kb_project(triple(ocra_filling_a_tray:'PlaceTokensInDescendingOrder', ocra:isPlanExecutedIn, ocra_filling_a_tray:'Event_58') during [800.0, 849.0]).
kb_project(triple(ocra_filling_a_tray:'PlaceTokensInDescendingOrder', dul:hasComponent, ocra_filling_a_tray:'FullBoardWithTokensInDescendingOrder') during [800.0, 849.0]).
kb_project(triple(ocra_filling_a_tray:'FullBoardWithTokensInDescendingOrder', dul:isComponentOf, ocra_filling_a_tray:'PlaceTokensInDescendingOrder') during [800.0, 849.0]).
kb_project(triple(ocra_filling_a_tray:'Event_58', dul:hasParticipant, ocra_filling_a_tray:'Robot') during [800.0, 849.0]).
kb_project(triple(ocra_filling_a_tray:'Robot', dul:isParticipantIn, ocra_filling_a_tray:'Event_58') during [800.0, 849.0]).
kb_project(triple(ocra_filling_a_tray:'Event_58', dul:hasParticipant, ocra_filling_a_tray:'Human') during [800.0, 849.0]).
kb_project(triple(ocra_filling_a_tray:'Human', dul:isParticipantIn, ocra_filling_a_tray:'Event_58') during [800.0, 849.0]).

kb_project(triple(ocra_filling_a_tray:'PlaceTokensInDescendingOrder', ocra:isPlanOf, ocra_filling_a_tray:'Robot') during [800.0, 849.0]).
kb_project(triple(ocra_filling_a_tray:'Robot', ocra:hasPlan, ocra_filling_a_tray:'PlaceTokensInDescendingOrder') during [800.0, 849.0]).
kb_project(triple(ocra_filling_a_tray:'PlaceTokensInDescendingOrder', ocra:isPlanOf, ocra_filling_a_tray:'Human') during [800.0, 849.0]).
kb_project(triple(ocra_filling_a_tray:'Human', ocra:hasPlan, ocra_filling_a_tray:'PlaceTokensInDescendingOrder') during [800.0, 849.0]).

kb_project(triple(ocra_filling_a_tray:'FullBoardWithTokensInDescendingOrder', ocra:isGoalOf, ocra_filling_a_tray:'Robot') during [800.0, 849.0]).
kb_project(triple(ocra_filling_a_tray:'Robot', ocra:hasGoal, ocra_filling_a_tray:'FullBoardWithTokensInDescendingOrder') during [800.0, 849.0]).

kb_project(triple(ocra_filling_a_tray:'Event_58', dul:hasLocation, ocra_filling_a_tray:'pmLab') during [800.0, 849.0]).
kb_project(triple(ocra_filling_a_tray:'pmLab', dul:isLocationOf, ocra_filling_a_tray:'Event_58') during [800.0, 849.0]).
kb_project(instance_of(ocra_filling_a_tray:'pmLab', ocra:'CollaborationPlace') during [800.0, 849.0]).



: : : : ADAPTATION _ DescendingFullCompartment : : 

kb_project(instance_of(ocra_filling_a_tray:'Event_63', ocra:'PlanAdaptation') during [850.0, 868.0]).
kb_project(triple(ocra_filling_a_tray:'Event_63', dul:hasParticipant, ocra_filling_a_tray:'Robot') during [850.0, 868.0]).
kb_project(triple(ocra_filling_a_tray:'Robot', dul:isParticipantIn, ocra_filling_a_tray:'Event_63') during [850.0, 868.0]).
kb_project(instance_of(ocra_filling_a_tray:'TargetCompartmentIsFull', dul:'Situation') during [858.0, 858.5]).
kb_project(triple(ocra_filling_a_tray:'TargetCompartmentIsFull', dul:isPostconditionOf, ocra_filling_a_tray:'ExecutionOfPlaceTokenOnCompartment18') during [858.0, 868.0]).
kb_project(triple(ocra_filling_a_tray:'ExecutionOfPlaceTokenOnCompartment18', dul:isPreconditionOf, ocra_filling_a_tray:'TargetCompartmentIsFull') during [858.0, 868.0]).
kb_project(triple(ocra_filling_a_tray:'TargetCompartmentIsFull', dul:isPreconditionOf, ocra_filling_a_tray:'ExecutionOfPlaceTokenOnCompartment19') during [858.5, 868.0]).
kb_project(triple(ocra_filling_a_tray:'ExecutionOfPlaceTokenOnCompartment19', dul:isPostconditionOf, ocra_filling_a_tray:'TargetCompartmentIsFull') during [858.5, 868.0]).

kb_project(triple(ocra_filling_a_tray:'ExecutionOfPlaceTokenOnCompartment18', dul:isPartOf, ocra_filling_a_tray:'Event_63') during [850.0, 858.0]).
kb_project(triple(ocra_filling_a_tray:'Event_63', dul:hasPart, ocra_filling_a_tray:'ExecutionOfPlaceTokenOnCompartment18') during [850.0, 858.0]).
kb_project(triple(ocra_filling_a_tray:'ExecutionOfPlaceTokenOnCompartment19', dul:isPartOf, ocra_filling_a_tray:'Event_63') during [858.5, 868.0]).
kb_project(triple(ocra_filling_a_tray:'Event_63', dul:hasPart, ocra_filling_a_tray:'ExecutionOfPlaceTokenOnCompartment19') during [858.5, 868.0]).

kb_project(triple(ocra_filling_a_tray:'ExecutionOfPlaceTokenOnCompartment18', ocra:executesPlan, ocra_filling_a_tray:'PlaceTokenOnCompartment18') during [850.0, 858.0]).
kb_project(triple(ocra_filling_a_tray:'PlaceTokenOnCompartment18', ocra:isPlanExecutedIn, ocra_filling_a_tray:'ExecutionOfPlaceTokenOnCompartment18') during [850.0, 858.0]).
kb_project(triple(ocra_filling_a_tray:'ExecutionOfPlaceTokenOnCompartment19', ocra:executesPlan, ocra_filling_a_tray:'PlaceTokenOnCompartment19') during [858.5, 868.0]).
kb_project(triple(ocra_filling_a_tray:'PlaceTokenOnCompartment19', ocra:isPlanExecutedIn, ocra_filling_a_tray:'ExecutionOfPlaceTokenOnCompartment19') during [858.5, 868.0]).
kb_project(instance_of(ocra_filling_a_tray:'_:DescriptionRobotA6a', owl:'NegativePropertyAssertion') during [858.5, 868.0]), kb_project(triple(ocra_filling_a_tray:'_:DescriptionRobotA6a', owl:'sourceIndividual', ocra_filling_a_tray:'ExecutionOfPlaceTokenOnCompartment19') during [858.5, 868.0]), kb_project(triple(ocra_filling_a_tray:'_:DescriptionRobotA6a', owl:'assertionProperty', ocra:'executesPlan') during [858.5, 868.0]), kb_project(triple(ocra_filling_a_tray:'_:DescriptionRobotA6a', owl:'targetIndividual', ocra_filling_a_tray:'PlaceTokenOnCompartment18') during [858.5, 868.0]).
kb_project(instance_of(ocra_filling_a_tray:'_:DescriptionRobotA6b', owl:'NegativePropertyAssertion') during [858.5, 868.0]), kb_project(triple(ocra_filling_a_tray:'_:DescriptionRobotA6b', owl:'sourceIndividual', ocra_filling_a_tray:'PlaceTokenOnCompartment18') during [858.5, 868.0]), kb_project(triple(ocra_filling_a_tray:'_:DescriptionRobotA6b', owl:'assertionProperty', ocra:'isPlanExecutedIn') during [858.5, 868.0]), kb_project(triple(ocra_filling_a_tray:'_:DescriptionRobotA6b', owl:'targetIndividual', ocra_filling_a_tray:'ExecutionOfPlaceTokenOnCompartment19') during [858.5, 868.0]).

kb_project(instance_of(ocra_filling_a_tray:'ExecutionOfPlaceTokenOnCompartment18', dul:'Event') during [850.0, 858.0]).
kb_project(instance_of(ocra_filling_a_tray:'ExecutionOfPlaceTokenOnCompartment19', dul:'Event') during [858.5, 868.0]).

kb_project(triple(ocra_filling_a_tray:'ExecutionOfPlaceTokenOnCompartment18', dul:hasParticipant, ocra_filling_a_tray:'Robot') during [850.0, 858.0]).
kb_project(triple(ocra_filling_a_tray:'Robot', dul:isParticipantIn, ocra_filling_a_tray:'ExecutionOfPlaceTokenOnCompartment18') during [850.0, 858.0]).
kb_project(triple(ocra_filling_a_tray:'ExecutionOfPlaceTokenOnCompartment19', dul:hasParticipant, ocra_filling_a_tray:'Robot') during [858.5, 868.0]).
kb_project(triple(ocra_filling_a_tray:'Robot', dul:isParticipantIn, ocra_filling_a_tray:'ExecutionOfPlaceTokenOnCompartment19') during [858.5, 868.0]).

kb_project(triple(ocra_filling_a_tray:'PlaceTokenOnCompartment18', ocra:isWorsePlanThan, ocra_filling_a_tray:'PlaceTokenOnCompartment19') during [858.5, 868.0]).
kb_project(triple(ocra_filling_a_tray:'PlaceTokenOnCompartment19', ocra:isBetterPlanThan, ocra_filling_a_tray:'PlaceTokenOnCompartment18') during [858.5, 868.0]).

kb_project(triple(ocra_filling_a_tray:'PlaceTokenOnCompartment18', dul:hasComponent, ocra_filling_a_tray:'FullBoardWithTokensInDescendingOrder') during [850.0, 868.0]).
kb_project(triple(ocra_filling_a_tray:'FullBoardWithTokensInDescendingOrder', dul:isComponentOf, ocra_filling_a_tray:'PlaceTokenOnCompartment18') during [850.0, 868.0]).
kb_project(triple(ocra_filling_a_tray:'PlaceTokenOnCompartment19', dul:hasComponent, ocra_filling_a_tray:'FullBoardWithTokensInDescendingOrder') during [858.5, 868.0]).
kb_project(triple(ocra_filling_a_tray:'FullBoardWithTokensInDescendingOrder', dul:isComponentOf, ocra_filling_a_tray:'PlaceTokenOnCompartment19') during [858.5, 868.0]).

kb_project(triple(ocra_filling_a_tray:'PlaceTokenOnCompartment18', ocra:isPlanOf, ocra_filling_a_tray:'Robot') during [850.0, 858.0]).
kb_project(triple(ocra_filling_a_tray:'Robot', ocra:hasPlan, ocra_filling_a_tray:'PlaceTokenOnCompartment18') during [850.0, 858.0]).
kb_project(triple(ocra_filling_a_tray:'PlaceTokenOnCompartment19', ocra:isPlanOf, ocra_filling_a_tray:'Robot') during [858.5, 868.0]).
kb_project(triple(ocra_filling_a_tray:'Robot', ocra:hasPlan, ocra_filling_a_tray:'PlaceTokenOnCompartment19') during [858.5, 868.0]).

: COLLABORATION BEHIND ADAPTATION
kb_project(instance_of(ocra_filling_a_tray:'Event_62', ocra:'Collaboration') during [850.0, 899.0]).
kb_project(triple(ocra_filling_a_tray:'Event_62', ocra:executesPlan, ocra_filling_a_tray:'PlaceTokensInDescendingOrder') during [850.0, 899.0]).
kb_project(triple(ocra_filling_a_tray:'PlaceTokensInDescendingOrder', ocra:isPlanExecutedIn, ocra_filling_a_tray:'Event_62') during [850.0, 899.0]).
kb_project(triple(ocra_filling_a_tray:'PlaceTokensInDescendingOrder', dul:hasComponent, ocra_filling_a_tray:'FullBoardWithTokensInDescendingOrder') during [850.0, 899.0]).
kb_project(triple(ocra_filling_a_tray:'FullBoardWithTokensInDescendingOrder', dul:isComponentOf, ocra_filling_a_tray:'PlaceTokensInDescendingOrder') during [850.0, 899.0]).
kb_project(triple(ocra_filling_a_tray:'Event_62', dul:hasParticipant, ocra_filling_a_tray:'Robot') during [850.0, 899.0]).
kb_project(triple(ocra_filling_a_tray:'Robot', dul:isParticipantIn, ocra_filling_a_tray:'Event_62') during [850.0, 899.0]).
kb_project(triple(ocra_filling_a_tray:'Event_62', dul:hasParticipant, ocra_filling_a_tray:'Human') during [850.0, 899.0]).
kb_project(triple(ocra_filling_a_tray:'Human', dul:isParticipantIn, ocra_filling_a_tray:'Event_62') during [850.0, 899.0]).

kb_project(triple(ocra_filling_a_tray:'PlaceTokensInDescendingOrder', ocra:isPlanOf, ocra_filling_a_tray:'Robot') during [850.0, 899.0]).
kb_project(triple(ocra_filling_a_tray:'Robot', ocra:hasPlan, ocra_filling_a_tray:'PlaceTokensInDescendingOrder') during [850.0, 899.0]).
kb_project(triple(ocra_filling_a_tray:'PlaceTokensInDescendingOrder', ocra:isPlanOf, ocra_filling_a_tray:'Human') during [850.0, 899.0]).
kb_project(triple(ocra_filling_a_tray:'Human', ocra:hasPlan, ocra_filling_a_tray:'PlaceTokensInDescendingOrder') during [850.0, 899.0]).

kb_project(triple(ocra_filling_a_tray:'FullBoardWithTokensInDescendingOrder', ocra:isGoalOf, ocra_filling_a_tray:'Robot') during [850.0, 899.0]).
kb_project(triple(ocra_filling_a_tray:'Robot', ocra:hasGoal, ocra_filling_a_tray:'FullBoardWithTokensInDescendingOrder') during [850.0, 899.0]).

kb_project(triple(ocra_filling_a_tray:'Event_62', dul:hasLocation, ocra_filling_a_tray:'pmLab') during [850.0, 899.0]).
kb_project(triple(ocra_filling_a_tray:'pmLab', dul:isLocationOf, ocra_filling_a_tray:'Event_62') during [850.0, 899.0]).
kb_project(instance_of(ocra_filling_a_tray:'pmLab', ocra:'CollaborationPlace') during [850.0, 899.0]).