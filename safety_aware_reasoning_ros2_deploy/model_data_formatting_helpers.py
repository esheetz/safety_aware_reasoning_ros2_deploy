"""
Model Data Formatting Helpers
Emily Sheetz, NSTGRO VTE, Summer 2024

note: for convenience, some of this is copied from the safety_aware_reasoning repo
i didn't feel like working out the dependencies or making a ROS2 version of that repo
so we're just copying and pasting for convenience
plus it makes this repo a little more minimal than the safety_aware_reasoning repo
"""

import os
import yaml

#######################
### DATASET HELPERS ###
#######################

def get_condition_risk_scores_for_robot_env(robot, env, risk_file):
    # create reader and process data
    state_space_reader = RiskyConditionReader(risky_condition_file=risk_file,
                                              robot=robot, environment=env)
    state_space_reader.process_risky_conditions()

    # initialize dictionary
    cond_risk_dict = {}

    # add risk scores to dictionary
    for cond in state_space_reader.get_risky_conditions():
        c_name = cond.get_condition_name()
        c_conseq = cond.get_consequence_class()
        c_risk = cond.get_matrix_risk_score()
        # add to dictionary
        cond_risk_dict[c_name] = {}
        cond_risk_dict[c_name]['consequence'] = c_conseq
        cond_risk_dict[c_name]['risk'] = c_risk

    return cond_risk_dict

###########################
### COLUMN NAME HELPERS ###
###########################

def check_col_name_for_condition_risk(col_name):
    return "COND_RISK_" in col_name

def check_col_name_for_state_conseq(col_name):
    return "STATE_CONSEQ" == col_name

def check_col_name_for_state_risk(col_name):
    return "STATE_RISK" == col_name

def get_condition_name_from_col_name(col_name):
    return col_name.replace("COND_RISK_","")

##########################
### STATE COMPUTATIONS ###
##########################

def compute_state_risk_score(condition_risk_scores):
    return max(condition_risk_scores)

def compute_state_consequence_score(condition_consequence_scores):
    return max(condition_consequence_scores)





#########################
### LIKELIHOOD LEVELS ###
#########################

class LikelihoodLevels:
    levels = {
        1 : "improbable",
        2 : "remote",
        3 : "occasional",
        4 : "probable",
        5 : "frequent",
    }
    min_level = 1
    max_level = 5

    @staticmethod
    def get_min():
        return LikelihoodLevels.min_level

    @staticmethod
    def get_max():
        return LikelihoodLevels.max_level

    @staticmethod
    def get_level_name(likelihood) -> str:
        # verify likelihood
        if likelihood not in LikelihoodLevels.levels.keys():
            print("ERROR: unrecognized likelihood level " + str(likelihood))
            return "unknown"

        return LikelihoodLevels.levels[likelihood]

    @staticmethod
    def valid_value(likelihood) -> bool:
        # valid likelihood values are ints and between bounds
        return ((type(likelihood) == int) and
                (LikelihoodLevels.get_min() <= likelihood) and
                (likelihood <= LikelihoodLevels.get_max()))

    @staticmethod
    def error_check(likelihood) -> int:
        # check if valid value received
        if LikelihoodLevels.valid_value(likelihood):
            # return un-modified likelihood
            return likelihood
        else:
            # check reason for invalid input
            if type(likelihood) != int:
                # invalid due to type
                print("ERROR: given likelihood value is of type " + str(type(likelihood)) + 
                      ", but expected type is int; " +
                      "setting likelihood value to max")
            else:
                # invalid due to bounds
                print("ERROR: given likelihood value " + str(likelihood) + " is outside bounds " +
                      "[" + str(LikelihoodLevels.get_min()) + "," + str(LikelihoodLevels.get_max()) + "]; " +
                      "setting likelihood value to max")
            # something was wrong with given value
            return LikelihoodLevels.get_max()



###########################
### CONSEQUENCE CLASSES ###
###########################

class ConsequenceClasses:
    classes = {
        1 : "insignificant",
        2 : "minor",
        3 : "moderate",
        4 : "major",
        5 : "severe",
    }
    min_level = 1
    max_level = 5

    @staticmethod
    def get_min():
        return ConsequenceClasses.min_level

    @staticmethod
    def get_max():
        return ConsequenceClasses.max_level

    @staticmethod
    def get_class_name(consequence) -> str:
        # verify consequence
        if consequence not in ConsequenceClasses.classes.keys():
            print("ERROR: unrecognized consequence class " + str(consequence))
            return "unknown"

        return ConsequenceClasses.classes[consequence]

    @staticmethod
    def valid_value(consequence) -> bool:
        # valid consequence values are ints and between bounds
        return ((type(consequence) == int) and
                (ConsequenceClasses.get_min() <= consequence) and
                (consequence <= ConsequenceClasses.get_max()))

    @staticmethod
    def error_check(consequence) -> int:
        # check if valid value received
        if ConsequenceClasses.valid_value(consequence):
            # return un-modified consequence
            return consequence
        else:
            # check reason for invalid input
            if type(consequence) != int:
                # invalid due to type
                print("ERROR: given consequence value is of type " + str(type(consequence)) + 
                      ", but expected type is int; " +
                      "setting consequence value to max")
            else:
                # invalid due to bounds
                print("ERROR: given consequence value " + str(consequence) + " is outside bounds " +
                      "[" + str(ConsequenceClasses.get_min()) + "," + str(ConsequenceClasses.get_max()) + "]; " +
                      "setting consequence value to max")
            # something was wrong with the given value
            return ConsequenceClasses.get_max()



###################
### RISK SCORES ###
###################

class RiskScores:
    scores = {
        # risk levels are of form (lower_bound, upper_bound) : "name"
        # with inclusive lower_bound and exclusive upper_bound
        (  1,  3 ) : "negligible",
        (  3,  4 ) : "low",
        (  4, 10 ) : "medium",
        ( 10, 17 ) : "high",
        ( 17, 26 ) : "extreme",
    }
    min_score = LikelihoodLevels.get_min() * ConsequenceClasses.get_min()
    max_score = LikelihoodLevels.get_max() * ConsequenceClasses.get_max()

    @staticmethod
    def get_min():
        return RiskScores.min_score

    @staticmethod
    def get_max():
        return RiskScores.max_score

    @staticmethod
    def get_raw_score_name(risk : int) -> str:
        # verify risk
        if (risk < RiskScores.get_min()) or (RiskScores.get_max() < risk):
            print("ERROR: unrecognized risk score " + str(risk))
            return "unknown"

        # find appropriate bounds
        for bounds in RiskScores.scores.keys():
            lower_bound, upper_bound = bounds
            if (lower_bound <= risk) and (risk < upper_bound):
                return RiskScores.scores[bounds]

        # should return at this point, but just in case
        return "unknown"

    @staticmethod
    def get_matrix_score_name(risk : float) -> str:
        # un-normalize risk
        raw_risk = int(risk * RiskScores.get_max())

        return RiskScores.get_raw_score_name(raw_risk)

    @staticmethod
    def get_score_name(likelihood : int, consequence : int) -> str:
        # compute risk according to risk assessment matrix
        matrix_risk = RiskScores.compute_matrix_risk_score(likelihood, consequence)

        return RiskScores.get_matrix_score_name(matrix_risk)

    @staticmethod
    def compute_raw_risk_score(likelihood : int, consequence : int) -> int:
        # error check
        likelihood = LikelihoodLevels.error_check(likelihood)
        consequence = ConsequenceClasses.error_check(consequence)

        return likelihood * consequence

    @staticmethod
    def compute_matrix_risk_score(likelihood : int, consequence : int) -> float:
        raw_risk = RiskScores.compute_raw_risk_score(likelihood, consequence)

        # normalized risk score
        risk = raw_risk / RiskScores.get_max()

        return risk

    @staticmethod
    def compute_risk_score(likelihood : int, consequence : int) -> float:
        # error check
        likelihood = LikelihoodLevels.error_check(likelihood)
        consequence = ConsequenceClasses.error_check(consequence)

        # turn likelihood into probability in [0,1]
        norm_likelihood = likelihood / LikelihoodLevels.get_max()

        return norm_likelihood * consequence

    @staticmethod
    def compute_matrix_safety_score(likelihood : int, consequence : int) -> float:
        # compute risk
        risk = RiskScores.compute_matrix_risk_score(likelihood, consequence)

        # compute safety
        safety = 1.0 - risk

        return safety

    @staticmethod
    def compute_safety_score(likelihood : int, consequence : int) -> float:
        # compute risk
        risk = RiskScores.compute_risk_score(likelihood, consequence)

        # compute safety
        safety = 1 / risk

        return safety



###################################
### YAML FILE FORMATTING CHECKS ###
###################################

class YAMLChecks:

    # CHECK YAML FILE EXISTENCE

    @staticmethod
    def check_yaml_existence(yaml_file_path):
        # check if given YAML file exists and is a file
        return (os.path.exists(yaml_file_path) and
                os.path.isfile(yaml_file_path))

    # CHECK YAML FORMATTING

    @staticmethod
    def check_yaml_formatting(file_nickname, yaml_dict, env_name, list_key, list_elem_keys):
        # check if environment exists
        if env_name not in yaml_dict.keys():
            print("ERROR: environment " + env_name + " does not exist in " + file_nickname + " file")
            return False

        # check for list key
        if list_key not in yaml_dict[env_name].keys():
            print("ERROR: environment " + env_name + " has no " + file_nickname + "s defined under key '" + list_key + "'")
            return False

        # get number of elements in list
        num_elems = len(yaml_dict[env_name][list_key])

        # initialize valid elements flag
        valid_elems = True

        # check each element in list
        for i in range(num_elems):
            # get element
            elem = yaml_dict[env_name][list_key][i]

            # check for each key in list element
            for elem_key in list_elem_keys:
                if elem_key not in elem.keys():
                    print("ERROR: " + file_nickname + " " + str(i) + " of " + str(num_elems) + " does not have key " + elem_key)
                    valid_elems = False

        return valid_elems



#####################################
### STATE SPACE FORMATTING CHECKS ###
#####################################

class YAMLStateSpaceChecks(YAMLChecks):

    @staticmethod
    def check_consequence_state_yaml_formatting(yaml_dict, env_name):
        return YAMLChecks.check_yaml_formatting(file_nickname="consequence state",
                                                yaml_dict=yaml_dict,
                                                env_name=env_name,
                                                list_key="consequences",
                                                list_elem_keys=["name"])

    @staticmethod
    def check_valid_consequence_state_values(state_dict, i, num_states):
        # initialize valid values flag
        valid_values = True

        # check for valid name
        if not type(state_dict['name']) == str:
            print("WARN: non-string name for consequence state " + str(i) + " of " + str(num_states))
            valid_values = False

        return valid_values

    @staticmethod
    def check_risky_condition_yaml_formatting(yaml_dict, env_name):
        return YAMLChecks.check_yaml_formatting(file_nickname="risky condition",
                                                yaml_dict=yaml_dict,
                                                env_name=env_name,
                                                list_key="conditions",
                                                list_elem_keys=["name","likelihood","consequence","consequence_states"])

    @staticmethod
    def check_valid_risky_condition_values(cond_dict, i, num_conds):
        # initialize valid values flag
        valid_values = True

        # check for valid name
        if not type(cond_dict['name']) == str:
            print("WARN: non-string name for risky condition " + str(i) + " of " + str(num_conds))
            valid_values = False

        # check for valid consequence states
        if not type(cond_dict['consequence_states']) == list:
            print("WARN: non-list consequence states for risky condition " + str(i) + " of " + str(num_conds))
            valid_values = False

        # check for valid likelihood and consequence scores
        if not LikelihoodLevels.valid_value(cond_dict['likelihood']):
            print("WARN: invalid likelihood value for risky condition " + str(i) + " of " + str(num_conds))
            valid_values = False

        if not ConsequenceClasses.valid_value(cond_dict['consequence']):
            print("WARN: invalid consequence value for risky condition " + str(i) + " of " + str(num_conds))
            valid_values = False

        return valid_values



#######################
### RISKY CONDITION ###
#######################

class RiskyCondition:
    def __init__(self, name="unnamed_condition",
                       likelihood=LikelihoodLevels.get_max(),
                       consequence=ConsequenceClasses.get_max(),
                       consequence_states=[]):
        # set internal parameters
        self.name = str(name)
        self.consequence_states = tuple(sorted([str(i) for i in consequence_states]))

        # set given likelihood and consequence values
        self.set_likelihood_level(likelihood)
        self.set_consequence_class(consequence)

        # compute corresponding risk and safety scores
        self.compute_risk_score()
        self.compute_safety_score()

    #######################
    ### GETTERS/SETTERS ###
    #######################

    def get_condition_name(self):
        return self.name

    def get_consequence_states(self):
        return self.consequence_states

    def set_likelihood_level(self, likelihood):
        # error check likelihood value
        self.likelihood = LikelihoodLevels.error_check(likelihood)
        return

    def get_likelihood_level(self):
        return self.likelihood

    def get_likelihood_level_name(self):
        return LikelihoodLevels.get_level_name(self.likelihood)

    def set_consequence_class(self, consequence):
        # error check consequence value
        self.consequence = ConsequenceClasses.error_check(consequence)
        return

    def get_consequence_class(self):
        return self.consequence

    def get_consequence_class_name(self):
        return ConsequenceClasses.get_class_name(self.consequence)

    def get_risk_score(self):
        return self.risk_score

    def get_risk_score_name(self):
        return self.risk_score_name

    def get_matrix_risk_score(self):
        return self.matrix_risk_score

    def get_matrix_risk_score_name(self):
        return self.matrix_risk_score_name

    def get_safety_score(self):
        return self.safety_score

    def get_matrix_safety_score(self):
        return self.matrix_safety_score

    ##########################
    ### RISK/SAFETY SCORES ###
    ##########################

    def compute_risk_score(self):
        # compute risk score
        self.risk_score = RiskScores.compute_risk_score(self.likelihood, self.consequence)
        # get corresponding risk score name
        self.risk_score_name = RiskScores.get_score_name(self.likelihood, self.consequence)
        # compute (somewhat more interpretable) risk assessment matrix score and name
        self.matrix_risk_score = RiskScores.compute_matrix_risk_score(self.likelihood, self.consequence)
        self.matrix_risk_score_name = RiskScores.get_matrix_score_name(self.matrix_risk_score)
        return

    def compute_safety_score(self):
        # compute safety score [0,1) based on risk score
        self.safety_score = RiskScores.compute_safety_score(self.likelihood, self.consequence)
        # compute (somewhat more interpretable) safety score
        self.matrix_safety_score = RiskScores.compute_matrix_safety_score(self.likelihood, self.consequence)
        return

    ################################################
    ### VALIDATE AGAINST CONSEQUENCE STATE SPACE ###
    ################################################

    def validate_condition(self, consequence_space_names):
        for state in self.consequence_states:
            if state not in consequence_space_names:
                print("ERROR: consequence " + state + " not in consequence state space: ", consequence_space_names)
                return False
        # if we get here, every consequence exists in consequence space
        return True

##############################
### RISKY CONDITION READER ###
##############################

class RiskyConditionReader:
    def __init__(self, risky_condition_file, robot, environment):
        # set internal parameters
        self.robot_name = robot
        self.environment_name = environment
        self.risky_condition_full_path = risky_condition_file

        # initialize list of risky conditions
        self.risky_conditions = []

        # initialize flag for valid conditions
        self.valid_conditions = False

    #######################
    ### GETTERS/SETTERS ###
    #######################

    def get_robot_name(self):
        return self.robot_name

    def get_environment_name(self):
        return self.environment_name

    def get_risky_condition_file_path(self):
        return self.risky_condition_full_path

    def get_risky_conditions(self):
        return self.risky_conditions

    def get_risky_condition_names(self):
        return [cond.get_condition_name() for cond in self.risky_conditions]

    def get_risky_condition_with_name(self, condition_name):
        # look through risky conditions
        for cond in self.risky_conditions:
            # check name
            if cond.get_condition_name() == condition_name:
                return cond
        # if we get here, no condition in list has given name
        return None

    def get_num_risky_conditions(self):
        return len(self.risky_conditions)

    ################################
    ### PROCESS RISKY CONDITIONS ###
    ################################

    def process_risky_conditions(self):
        # clear out risky conditions list
        self.risky_conditions = []

        # verify YAML file exists
        valid_path = YAMLStateSpaceChecks.check_yaml_existence(self.risky_condition_full_path)
        if not valid_path:
            print("ERROR: risky condition file " + self.risky_condition_full_path + " does not exist")
            self.valid_conditions = False
            return

        # open YAML file and load dict
        fo = open(self.risky_condition_full_path)
        yaml_dict = yaml.load(fo, Loader=yaml.FullLoader)

        # error check YAML file formatting
        valid_yaml = YAMLStateSpaceChecks.check_risky_condition_yaml_formatting(yaml_dict, self.environment_name)
        if not valid_yaml:
            print("ERROR: risky condition file " + self.risky_condition_full_path + " is poorly formatted")
            self.valid_conditions = False
            return

        # initialize valid conditions flag
        self.valid_conditions = True

        # get list of conditions for environment
        conditions = yaml_dict[self.environment_name]['conditions']

        # process each condition
        for i in range(len(conditions)):
            # get condition
            cond = conditions[i]

            # check valid values for risky condition
            valid_condition = YAMLStateSpaceChecks.check_valid_risky_condition_values(cond, i, len(conditions))
            self.valid_conditions = self.valid_conditions and valid_condition

            # create risky condition
            risky_cond = RiskyCondition(name=cond['name'],
                                        likelihood=cond['likelihood'],
                                        consequence=cond['consequence'],
                                        consequence_states=cond['consequence_states'])

            # add risky condition to list
            self.risky_conditions.append(risky_cond)

        # close file
        fo.close()

        return

    def check_valid_conditions(self):
        return self.valid_conditions
