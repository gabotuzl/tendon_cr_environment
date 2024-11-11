__doc__ = """ Numba implementation module for boundary condition implementations that apply
external forces to the system."""

import numpy as np
from elastica.typing import SystemType, RodType

# Normally NoForces would be included in the forcing module, but PyElastica requires it to be from the original elastica.external_forces module
from elastica.external_forces import NoForces
from numba import njit


class TendonForces(NoForces):
    """
    This class applies tendon forcing along the length of the rod.

        Attributes
        ----------
        vertebra_height: float
            Height at which the tendon contacts the vertebra. It should be the highest point on the tendon-vertebra space.
        num_vertebrae: int
            Amount of vertebrae to be used in the system.
        vertebra_height_vector: numpy.ndarray
            1D (dim) numpy array. Describes the orientation and height in space of the vertebrae in the system.
        tension: float
            Tension applied to the tendon in the system.
        n_elements: int
            Total amount of nodes in the rod system. This value is set in the simulator and is copied to this class for later use.
        vertebra_weight_vector: numpy.ndarray
            1D (dim) numpy array. Vector which specifies the orientation and magnitude of the weight of the vertebrae (By default it is in the global -Z direction).
        vertebra_nodes: list
            1D (dim) list. Contains the node numbers of every node with vertebrae. The vertebrae are assumed to be uniformly spaced through the intervals specified by 
            first_vertebra_node and final_vertebra_node, with an amount equal to num_vertebrae.
        force_data: numpy.ndarray
            2D (dim,3) numpy array. Contains the force vectors caused by tendon forcing for each of the nodes with vertebrae.
        

    """

    def __init__(self, vertebra_height, num_vertebrae, first_vertebra_node, final_vertebra_node, vertebra_mass, tension, vertebra_height_orientation, n_elements):
        """

        Parameters 
        ----------
        vertebra_height: float
            Height at which the tendon contacts the vertebra. It should be the highest point on the tendon-vertebra space.
        num_vertebrae: int
            Amount of vertebrae to be used in the system.
        first_vertebra_node: int
            The first node to have a vertebra, from the base of the rod to the tip.
        final_vertebra_node: int
            The last node to have a vertebra, from the base of the rod to the tip.
        vertebra_mass: float
            Total mass of a single vertebra.
        tension: float
            Tension applied to the tendon in the system.
        vertebra_height_orientation: numpy.ndarray
            1D (dim) numpy array. Describes the orientatation of the vertebrae in the system.
        n_elements: int
            Total amount of nodes in the rod system. This value is set in the simulator and is copied to this class for later use.
        """
        super(TendonForces, self).__init__()

        # Initializing class attributes to be used in other methods
        self.vertebra_height = vertebra_height
        self.num_vertebrae = num_vertebrae
        self.vertebra_height_vector = vertebra_height_orientation * vertebra_height
        self.tension = tension
        self.n_elements = n_elements
        self.vertebra_weight_vector = np.array([0.0, 0.0, -vertebra_mass * 9.80665])

        # Creating vector containing the node numbers with the vertebras for this instance of TendonForces
        self.vertebra_nodes = []
        vertebra_increment = (final_vertebra_node - first_vertebra_node)/(num_vertebrae - 1)
        for i in range(num_vertebrae):
            self.vertebra_nodes.append(round(i * vertebra_increment + first_vertebra_node))

    def apply_forces(self, system: SystemType, time: np.float64 = 0.0):
        # The application of the force data is done outside of the @njit decorated function because self.force_data needs to be referenced in self.compute_torques()

        # Retrieves relative position unit norm vectors between each vertebra top (where the tendon contacts the vertebra)
        unit_norm_vector_array = self.get_rotations(np.array(system.position_collection), np.array(system.director_collection), np.array(self.vertebra_nodes), self.vertebra_height_vector)

        # Computes the forces in each vertebra
        self.force_data = self.compute_forces(self.tension, np.array(self.vertebra_nodes), unit_norm_vector_array)

        # Creating the force data set to apply to the rod
        apply_force = np.zeros((3,self.n_elements+1))

        # PyElastica handles forces in GLOBAL coord. system, so they are applied directly. Also, the vertebra weights are added to each vertebra
        for i in range (len(self.vertebra_nodes)):
            apply_force[:,self.vertebra_nodes[i]] = self.force_data[i] + self.vertebra_weight_vector

        # Applies forces to the rod
        system.external_forces += apply_force


    def apply_torques(self, system: SystemType, time: np.float64 = 0.0):
        # The force_data set and vertebra_weight_vector are expressed in the global coordinate frame and must be changed to local reference frames for torque application
        # Creating the array which will contain the transformed force vectors
        transformed_force_data = np.zeros((len(self.vertebra_nodes), 3), dtype=np.float64)

        # Transforming the force vectors calculated in the compute_forces method from the global reference frame to the local reference frame
        for i in range(len(self.vertebra_nodes)):
            transformed_force_data[i] = system.director_collection[...,(self.vertebra_nodes[i]-1)] @ self.force_data[i]

        self.compute_torques(
            self.vertebra_height_vector, np.array(self.vertebra_nodes), transformed_force_data,
            self.n_elements, system.external_torques
        )


    @staticmethod
    @njit(cache=True)
    def get_rotations(position_collection, director_collection, vertebra_nodes, vertebra_height_vector):
        # Returns an array containing the unit norm vector which describes the orientation of each segment of tendon between vertebrae

        # Initializing unit_norm_vector_array to store the unit normed vectors that describe the global orientation of the forces in each vertebra
        unit_norm_vector_array = np.zeros((len(vertebra_nodes), 3), dtype=np.float64)

        for i in range(len(vertebra_nodes)+1):
            # There is a +1 in the for loop to account for the force between the first vertebra and the fixed node

            # If statement, used for the case when i = 0 and thus there is no vertebra before this one, same for the final vertebra (no vertebra after that one)
            if i==0:
                current_vertebra = 0
                next_vertebra = vertebra_nodes[i]
            elif i==len(vertebra_nodes):
                current_vertebra = vertebra_nodes[i-1]
                next_vertebra = vertebra_nodes[i-1]
            else:
                current_vertebra = vertebra_nodes[i-1]
                next_vertebra = vertebra_nodes[i]

            # Setting up values to be used iteratively
            x_current = position_collection[0, current_vertebra]
            y_current = position_collection[1, current_vertebra]
            z_current = position_collection[2, current_vertebra]

            x_next = position_collection[0, next_vertebra]
            y_next = position_collection[1, next_vertebra]
            z_next = position_collection[2, next_vertebra]

            current_rotation_matrix = director_collection[...,current_vertebra]
            next_rotation_matrix = director_collection[...,next_vertebra]

            current_node = np.array([x_current, y_current, z_current])
            next_node = np.array([x_next, y_next, z_next])

            # Calculating relative position vector between vertebrae, considering the vertebra height
            # Continguous arrays to help with computation speed
            delta_vector = (next_node + np.ascontiguousarray(next_rotation_matrix.T) @ np.ascontiguousarray(vertebra_height_vector)) - (current_node + np.ascontiguousarray(current_rotation_matrix.T) @ np.ascontiguousarray(vertebra_height_vector))

            # Calculating the unit-normed vector based on the differences calculated in the previous step
            delta_vector_norm = np.linalg.norm(delta_vector)
            unit_norm_delta_vector = delta_vector / delta_vector_norm

            # This if statement is to stop unit_norm_delta_vector from becoming a 'nan'
            if i==len(vertebra_nodes):
                unit_norm_delta_vector = np.zeros(3)

            # Storing the unit normed vector to be later used in the compute_forces method
            unit_norm_vector_array[i] = unit_norm_delta_vector

        return unit_norm_vector_array

    @staticmethod
    @njit(cache=True)
    def compute_forces(tension, vertebra_nodes, unit_norm_vector_array):

        # Creating array to store forces in vertebrae
        force_data = np.zeros((len(vertebra_nodes), 3), dtype=np.float64)

        for i in range(len(vertebra_nodes)):
            # This for loop multiplies the unit normed vectors calculated previously, with the tension of the tendon, thus creating the force vector for each vertebra
            # Contiguous array to increase speed in njit decorator
            force_current_prev = unit_norm_vector_array[i] * -tension
            force_current_next = unit_norm_vector_array[i+1] * tension

            # Summing the components of both force vectors to get the final force vector, which is then stored for use in the apply_forces and compute_torques methods
            force_data[i] = force_current_prev + force_current_next

        return force_data


    @staticmethod
    @njit(cache=True)
    def compute_torques(vertebra_height_vector, vertebra_nodes, transformed_force_data, n_elements, external_torques):

        # Creating torque data set for storage
        torque_data = np.zeros((len(vertebra_nodes), 3),dtype=np.float64)

        # Goes through vertebra nodes to calculate torques for them
        for i in range(len(vertebra_nodes)):

            # Cross product between the vertebra height vector and the local force vector due to the tendons, to obtain the tendon torque for that vertebra
            torque_vector = np.cross(vertebra_height_vector, transformed_force_data[i])

            # Sum of the vectors, and storage into the torque_data array
            torque_data[i] = torque_vector

        # Appending the computed torque vector to the final torque data set
        apply_torque = np.zeros((3,n_elements+1))

        k = 0
        for i in range(n_elements):
            if i in vertebra_nodes:
                apply_torque[:,i] = torque_data[k]
                k += 1
        apply_torque = apply_torque[:,1:]

        # Applying the torque data set to the rod (torque on the final vertebra)
        external_torques += apply_torque

class QuadTendonForces(NoForces):
    """
    This class uses a Quad Tendon Configuration. Its purpose is to allow for closed loop control of the tip of the rod through tendon actuation.
    Quad Tendon Configuration: Tendon configuration which has 4 long tendons (up, down, left, right) and 4 short tendons (up, down, left, right) to allow for the simultaneous 
                               activation of multiple tendons to reach points in the 3D workspace of the continuum robot.
                              
        Attributes
        ----------
        vertebra_height_long: float
            Height at which the tendon contacts the vertebra. It should be the highest point on the tendon-vertebra space. This attribute relates to ALL LONG tendon systems.
        vertebra_height_short: numpy.ndarray
            Height at which the tendon contacts the vertebra. It should be the highest point on the tendon-vertebra space. This attribute relates to ALL SHORT tendon systems.
        num_vertebrae_long: int
            Amount of vertebrae to be used in the system. This attribute relates to ALL LONG vertebrae.
        num_vertebrae_short: int
            Amount of vertebrae to be used in the system. This attribute relates to ALL SHORT vertebrae.
        n_elements: int
            Total amount of nodes in the rod system. This value is set in the simulator and is copied to this class for later use.
        vertebra_weight_vector_long: numpy.ndarray
            1D (dim) numpy array. Vector which specifies the orientation and magnitude of the weight of the LONG vertebrae (By default it is in the global -Z direction).
        vertebra_weight_vector_short: numpy.ndarray
            1D (dim) numpy array. Vector which specifies the orientation and magnitude of the weight of the SHORT vertebrae (By default it is in the global -Z direction).
        vertebra_nodes_long: list
            1D (dim) list. Contains the node numbers of every node with LONG vertebrae. The vertebrae are assumed to be uniformly spaced through the intervals specified by 
            first_vertebra_node_long and final_vertebra_node_long, with an amount equal to num_vertebrae_long.
        vertebra_nodes_short:list
            1D (dim) list. Contains the node numbers of every node with SHORT vertebrae. The vertebrae are assumed to be uniformly spaced through the intervals specified by 
            first_vertebra_node_short and final_vertebra_node_short, with an amount equal to num_vertebrae_short.
        vertebra_height_vector_vertical_long: numpy.ndarray
            1D (dim) numpy array. Describes the orientation and height in space of the (ACTIVE) VERTICAL LONG vertebrae in the system.
        vertebra_height_vector_horizontal_long: numpy.ndarray
            1D (dim) numpy array. Describes the orientation and height in space of the (ACTIVE) HORIZONTAL LONG vertebrae in the system.
        vertebra_height_vector_vertical_short: numpy.ndarray
            1D (dim) numpy array. Describes the orientation and height in space of the (ACTIVE) VERTICAL SHORT vertebrae in the system.
        vertebra_height_vector_horizontal_short: numpy.ndarray
            1D (dim) numpy array. Describes the orientation and height in space of the (ACTIVE) HORIZONTAL SHORT vertebrae in the system.
        tension_vertical_long: float
            Tension applied to the (ACTIVE) VERTICAL LONG tendon in the system.
        tension_horizontal_long: float
            Tension applied to the (ACTIVE) HORIZONTAL LONG tendon in the system.
        tension_vertical_short: float
            Tension applied to the (ACTIVE) VERTICAL SHORT tendon in the system.
        tension_horizontal_short: float
            Tension applied to the (ACTIVE) HORIZONTAL SHORT tendon in the system.
        force_data_vertical_long: numpy.ndarray
            2D (dim,3) numpy array. Contains the force vectors caused by tendon forcing for each of the (ACTIVE) VERTICAL LONG vertebrae.
        force_data_horizontal_long: numpy.ndarray
            2D (dim,3) numpy array. Contains the force vectors caused by tendon forcing for each of the (ACTIVE) HORIZONTAL LONG vertebrae.
        force_data_vertical_short: numpy.ndarray
            2D (dim,3) numpy array. Contains the force vectors caused by tendon forcing for each of the (ACTIVE) VERTICAL SHORT vertebrae.
        force_data_horizontal_short: numpy.ndarray
            2D (dim,3) numpy array. Contains the force vectors caused by tendon forcing for each of the (ACTIVE) HORIZONTAL SHORT vertebrae.
    """

    def __init__(self, vertebra_height_long, num_vertebrae_long, first_vertebra_node_long, final_vertebra_node_long, vertebra_mass_long,
    vertebra_height_short, num_vertebrae_short, first_vertebra_node_short, final_vertebra_node_short, vertebra_mass_short, n_elements):
        """

        Parameters 
        ----------
        vertebra_height_long: float
            Height at which the tendon contacts the vertebra. It should be the highest point on the tendon-vertebra space. This parameter relates to ALL LONG tendon systems.
        num_vertebrae_long: int
            Amount of vertebrae to be used in the system. This relates to ALL LONG tendon systems.
        first_vertebra_node_long: int
            The first node to have a vertebra, from the base of the rod to the tip. This relates to ALL LONG tendon systems.
        final_vertebra_node_long: int
            The last node to have a vertebra, from the base of the rod to the tip. This relates to ALL LONG tendon systems.
        vertebra_mass_long: float
            Total mass of a single vertebra disk. This relates to ALL LONG tendon systems.
        vertebra_height_short: float
            Height at which the tendon contacts the vertebra. It should be the highest point on the tendon-vertebra space. This parameter relates to ALL SHORT tendon systems.
        num_vertebrae_short: int
            Amount of vertebrae to be used in the system. This relates to ALL SHORT tendon systems.
        first_vertebra_node_short: int
            The first node to have a vertebra, from the base of the rod to the tip. This relates to ALL SHORT tendon systems.
        final_vertebra_node_short: int
            The last node to have a vertebra, from the base of the rod to the tip. This relates to ALL SHORT tendon systems.
        vertebra_mass_short: float
            Total mass of a single vertebra disk. This relates to ALL SHORT tendon systems.
        n_elements: int
            Total amount of nodes in the rod system. This value is set in the simulator and is copied to this class for later use.
        """
        super(QuadTendonForces, self).__init__()

        # Initializing class attributes to be used in other methods
        self.vertebra_height_long = vertebra_height_long
        self.vertebra_height_short = vertebra_height_short
        self.num_vertebrae_long = num_vertebrae_long
        self.num_vertebrae_short = num_vertebrae_short
        self.n_elements = n_elements

        # Calculating the weight vectores for the vertebrae. By default, the direction of gravity is in the global -Z direction
        self.vertebra_weight_vector_long = np.array([0.0, 0.0, -vertebra_mass_long * 9.80665])
        self.vertebra_weight_vector_short = np.array([0.0, 0.0, -vertebra_mass_short * 9.80665])

        # Creating vector containing the node numbers with the vertebrae for the long tendon
        self.vertebra_nodes_long = []
        vertebra_increment_long = (final_vertebra_node_long - first_vertebra_node_long)/(num_vertebrae_long - 1)
        for i in range(num_vertebrae_long):
            self.vertebra_nodes_long.append(round(i * vertebra_increment_long + first_vertebra_node_long))

        # Creating vector containing the node numbers with the vertebrae for the short tendon
        self.vertebra_nodes_short = []
        vertebra_increment_short = (final_vertebra_node_short - first_vertebra_node_short)/(num_vertebrae_short - 1)
        for i in range(num_vertebrae_short):
            self.vertebra_nodes_short.append(round(i * vertebra_increment_short + first_vertebra_node_short))

        # Initializing all the attributes defined in the update_tendon_tension method, so that PyElastica can run even if the controller has not yet sent the parameters
        self.vertebra_height_vector_vertical_long = np.array([0.0, 0.0, 0.0])
        self.vertebra_height_vector_horizontal_long = np.array([0.0, 0.0, 0.0])
        self.vertebra_height_vector_vertical_short = np.array([0.0, 0.0, 0.0])
        self.vertebra_height_vector_horizontal_short = np.array([0.0, 0.0, 0.0])

        self.tension_vertical_long = 0.0
        self.tension_horizontal_long = 0.0
        self.tension_vertical_short = 0.0
        self.tension_horizontal_short = 0.0

    def update_tendon_tension(self, tendon_tensions):
        # Getting the control signal published by the control node and received through the '/tendon_tensions' topic
        # The negative sign is to match the horizontal orientations with the local reference frames, as message from '/tendon_tensions' is with respect to the global reference frame
        self.vertebra_height_vector_vertical_long = np.array([tendon_tensions[0], 0.0, 0.0]) * self.vertebra_height_long
        self.vertebra_height_vector_horizontal_long = np.array([0.0, -tendon_tensions[1], 0.0]) * self.vertebra_height_long
        self.vertebra_height_vector_vertical_short = np.array([tendon_tensions[2], 0.0, 0.0]) * self.vertebra_height_short
        self.vertebra_height_vector_horizontal_short = np.array([0.0, -tendon_tensions[3], 0.0]) * self.vertebra_height_short

        self.tension_vertical_long = tendon_tensions[4]
        self.tension_horizontal_long = tendon_tensions[5]
        self.tension_vertical_short = tendon_tensions[6]
        self.tension_horizontal_short = tendon_tensions[7]

        return

    def apply_forces(self, system: SystemType, time: np.float64 = 0.0):
        # The application of the force data is done outside of the @njit decorated function because self.force_data needs to be referenced in self.compute_torques()

        # Retrieves relative position unit norm vectors between each vertebra for the long and short tendons
        unit_norm_vector_array_vertical_long = self.get_rotations(np.array(system.position_collection), np.array(system.director_collection), np.array(self.vertebra_nodes_long), self.vertebra_height_vector_vertical_long)
        unit_norm_vector_array_horizontal_long = self.get_rotations(np.array(system.position_collection), np.array(system.director_collection), np.array(self.vertebra_nodes_long), self.vertebra_height_vector_horizontal_long)
        unit_norm_vector_array_vertical_short = self.get_rotations(np.array(system.position_collection), np.array(system.director_collection), np.array(self.vertebra_nodes_short), self.vertebra_height_vector_vertical_short)
        unit_norm_vector_array_horizontal_short = self.get_rotations(np.array(system.position_collection), np.array(system.director_collection), np.array(self.vertebra_nodes_short), self.vertebra_height_vector_horizontal_short)

        # Computes the forces in each vertebra
        self.force_data_vertical_long = self.compute_forces(self.tension_vertical_long, np.array(self.vertebra_nodes_long), unit_norm_vector_array_vertical_long)
        self.force_data_horizontal_long = self.compute_forces(self.tension_horizontal_long, np.array(self.vertebra_nodes_long), unit_norm_vector_array_horizontal_long)
        self.force_data_vertical_short= self.compute_forces(self.tension_vertical_short, np.array(self.vertebra_nodes_short), unit_norm_vector_array_vertical_short)
        self.force_data_horizontal_short = self.compute_forces(self.tension_horizontal_short, np.array(self.vertebra_nodes_short), unit_norm_vector_array_horizontal_short)

        # Creating the force data set to apply to the rod
        apply_force = np.zeros((3,self.n_elements+1))

        # PyElastica handles forces in GLOBAL coord. system, so they are applied directly. Also, the vertebra weights are added to each vertebra
        for i in range (len(self.vertebra_nodes_long)):
            apply_force[:,self.vertebra_nodes_long[i]] += self.force_data_vertical_long[i] + self.force_data_horizontal_long[i] + self.vertebra_weight_vector_long
        for i in range (len(self.vertebra_nodes_short)):
            apply_force[:,self.vertebra_nodes_short[i]] += self.force_data_vertical_short[i] + self.force_data_horizontal_short[i] + self.vertebra_weight_vector_short

        # Applies forces to the rod
        system.external_forces += apply_force

    def apply_torques(self, system: SystemType, time: np.float64 = 0.0):
        # The force_data set and vertebra_weight_vector are expressed in the global coordinate frame and must be changed to local reference frames for torque application
        # Creating the array which will contain the transformed force vectors
        transformed_force_data_vertical_long = np.zeros((len(self.vertebra_nodes_long), 3), dtype=np.float64)
        transformed_force_data_horizontal_long = np.zeros((len(self.vertebra_nodes_long), 3), dtype=np.float64)
        transformed_force_data_vertical_short = np.zeros((len(self.vertebra_nodes_short), 3), dtype=np.float64)
        transformed_force_data_horizontal_short = np.zeros((len(self.vertebra_nodes_short), 3), dtype=np.float64)

        # Transforming the force vectors calculated in the compute_forces method from the global reference frame to the local reference frame
        # Doing this for both vertical and horizontal tendons, of long and short lengths
        for i in range(len(self.vertebra_nodes_long)):
            transformed_force_data_vertical_long[i] = np.ascontiguousarray(system.director_collection[...,(self.vertebra_nodes_long[i]-1)]) @ np.ascontiguousarray(self.force_data_vertical_long[i])
            transformed_force_data_horizontal_long[i] = np.ascontiguousarray(system.director_collection[...,(self.vertebra_nodes_long[i]-1)]) @ np.ascontiguousarray(self.force_data_horizontal_long[i])

        for i in range(len(self.vertebra_nodes_short)):
            transformed_force_data_vertical_short[i] = np.ascontiguousarray(system.director_collection[...,(self.vertebra_nodes_short[i]-1)]) @ np.ascontiguousarray(self.force_data_vertical_short[i])
            transformed_force_data_horizontal_short[i] = np.ascontiguousarray(system.director_collection[...,(self.vertebra_nodes_short[i]-1)]) @ np.ascontiguousarray(self.force_data_horizontal_short[i])

        # Calculating torque vectors for vertebrae using both vertical and horizontal tendons, of long and short lengths
        apply_torque_long = self.compute_torques(
            self.vertebra_height_vector_vertical_long, self.vertebra_height_vector_horizontal_long, np.array(self.vertebra_nodes_long),
            transformed_force_data_vertical_long, transformed_force_data_horizontal_long, self.n_elements
        )

        apply_torque_short = self.compute_torques(
            self.vertebra_height_vector_vertical_short, self.vertebra_height_vector_horizontal_short, np.array(self.vertebra_nodes_short),
            transformed_force_data_vertical_short, transformed_force_data_horizontal_short, self.n_elements
        )

        # Applying the torque data set to the rod
        system.external_torques += apply_torque_long + apply_torque_short

    @staticmethod
    @njit(cache=True)
    def get_rotations(position_collection, director_collection, vertebra_nodes, vertebra_height_vector):
        # Returns an array containing the unit norm vector which describes the orientation of each segment of tendon between vertebrae

        # Initializing unit_norm_vector_array to store the unit normed vectors that describe the global orientation of the forces in each vertebra
        unit_norm_vector_array = np.zeros((len(vertebra_nodes), 3), dtype=np.float64)

        for i in range(len(vertebra_nodes)+1):
            # There is a +1 in the for loop to account for the force between the first vertebra and the fixed node

            # If statement, used for the case when i = 0 and thus there is no vertebra before this one, same for the final vertebra (no vertebra after that one)
            if i==0:
                current_vertebra = 0
                next_vertebra = vertebra_nodes[i]
            elif i==len(vertebra_nodes):
                current_vertebra = vertebra_nodes[i-1]
                next_vertebra = vertebra_nodes[i-1]
            else:
                current_vertebra = vertebra_nodes[i-1]
                next_vertebra = vertebra_nodes[i]

            # Setting up values to be used iteratively
            x_current = position_collection[0, current_vertebra]
            y_current = position_collection[1, current_vertebra]
            z_current = position_collection[2, current_vertebra]

            x_next = position_collection[0, next_vertebra]
            y_next = position_collection[1, next_vertebra]
            z_next = position_collection[2, next_vertebra]

            current_rotation_matrix = director_collection[...,current_vertebra]
            next_rotation_matrix = director_collection[...,next_vertebra]

            current_node = np.array([x_current, y_current, z_current])
            next_node = np.array([x_next, y_next, z_next])

            # Calculating relative position vector between vertebrae, considering the vertebra height
            delta_vector = (next_node + np.ascontiguousarray(next_rotation_matrix.T) @ np.ascontiguousarray(vertebra_height_vector)) - (current_node + np.ascontiguousarray(current_rotation_matrix.T) @ np.ascontiguousarray(vertebra_height_vector))

            # Calculating the unit-normed vector based on the differences calculated in the previous step
            delta_vector_norm = np.linalg.norm(delta_vector)
            unit_norm_delta_vector = delta_vector / delta_vector_norm

            # This if statement is to stop unit_norm_delta_vector from becoming a 'nan'
            if i==len(vertebra_nodes):
                unit_norm_delta_vector = np.zeros(3)

            # Storing the unit normed vector to be later used in the compute_forces method
            unit_norm_vector_array[i] = unit_norm_delta_vector

        return unit_norm_vector_array


    @staticmethod
    @njit(cache=True)
    def compute_forces(tension, vertebra_nodes, unit_norm_vector_array):
        # Returns an array containing the resulting tendon force vectors for the vertebrae system being analyzed

        # Creating array to store forces in vertebrae
        force_data = np.zeros((len(vertebra_nodes), 3), dtype=np.float64)

        for i in range(len(vertebra_nodes)):
            # This for loop multiplies the unit normed vectors calculated previously, with the tension of each tendon, thus creating the force vector for each vertebra
            # Contiguous array to increase speed in njit decorator
            force_current_prev = unit_norm_vector_array[i] * -tension
            force_current_next = unit_norm_vector_array[i+1] * tension

            # Summing the components of both force vectors to get the final force vector, which is then stored for use in the apply_forces and compute_torques methods
            force_data[i] = force_current_prev + force_current_next

        return force_data

    @staticmethod
    @njit(cache=True)
    def compute_torques(vertebra_height_vector_vertical, vertebra_height_vector_horizontal, vertebra_nodes,
                        transformed_force_data_vertical, transformed_force_data_horizontal, n_elements):
        # Returns array containing tendon torques applied to respective vertebrae nodes in the format PyElastica uses for external forcing 

        # Creating torque data set for storage
        torque_data = np.zeros((len(vertebra_nodes), 3),dtype=np.float64)

        # Goes through vertebra nodes to calculate torques for them
        for i in range(len(vertebra_nodes)):

            # Cross product between the vertebra height vector and the local force vector due to the tendons, to obtain the tendon torque for that vertebra
            torque_vector_vertical = np.cross(vertebra_height_vector_vertical, transformed_force_data_vertical[i])
            torque_vector_horizontal = np.cross(vertebra_height_vector_horizontal, transformed_force_data_horizontal[i])

            # Sum of the vectors, and storage into the torque_data array
            torque_data[i] = torque_vector_vertical + torque_vector_horizontal

        # Appending the computed torque vector to the final torque data set
        apply_torque = np.zeros((3,n_elements+1))

        k = 0
        for i in range(n_elements):
            if i in vertebra_nodes:
                apply_torque[:,i] = torque_data[k]
                k += 1
        apply_torque = apply_torque[:,1:]

        return apply_torque

