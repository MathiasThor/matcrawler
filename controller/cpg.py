#!/usr/bin/env python
import numpy as np


class CPG(object):
    def __init__(self, new_phi):
        # Configuration parameters of the CPG
        self.phi    = new_phi
        self.alpha  = 1.01

        # Shape the Weight Matrix
        self.W_mat   = np.matrix([[np.cos(self.phi), np.sin(self.phi)], [-np.sin(self.phi), np.cos(self.phi)]]) * self.alpha

        # Initial configuration of the state of neuron output
        self.O_vec   = np.matrix([[-0.2012], [0.0]])

    def step(self):
        self.O_vec = np.tanh(self.W_mat * self.O_vec)

    def set_phi(self, new_phi):
        self.phi = new_phi

    def get_output(self, output):
        return self.O_vec[output].item()
