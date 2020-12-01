# -*- coding: utf-8 -*-
"""
Created on Mon Nov 23 09:45:06 2020

@author: fortineau_vin
"""

#%%
""" IMPORTS """

import matplotlib.pyplot as plt
from os import path
from os import getcwd
from torch.utils.data import DataLoader
import numpy as np
from lstm_trajectory_prediction_v1 import LightningLSTM
from lstm_trajectory_prediction_v1 import matlabDataPrePro
#import pickle
import scipy.io

#%%
""" MACROS """

#MODEL_REL_PATH = 'lightning_logs/version_8/checkpoints/force_estimation-epoch=25-val_loss=0.00.ckpt'
#MODEL_REL_PATH = 'lightning_logs/version_14/checkpoints/force_estimation-epoch=26-val_loss=0.00.ckpt'
MODEL_REL_PATH = 'lightning_logs/version_1/checkpoints/force_estimation-epoch=28-val_loss=0.02.ckpt'
INPUT_FILE_NAME = 'data_vfo_3_phases_subData'
SAVE_PREDICTIONS = True

#%%
""" CLASSES """

#%%
""" MAIN """

# Load Neural Network 
basepath = getcwd()
filepath = path.abspath(path.join(basepath, MODEL_REL_PATH))
model = LightningLSTM.load_from_checkpoint(filepath, batch_size=1)

# Load test data
filepath = path.abspath(path.join(basepath, "..", "experimental_bench_pert", "preliminary_experimental_data", INPUT_FILE_NAME))
test = matlabDataPrePro(filepath, 'subDataTest', normalization="Variance", mask_size=200)  
test_loader = DataLoader(test, batch_size=1)

test_loss = list()

data_prediction = list()

for item in test_loader:
  x, y = item
  y_hat = model(x)
  y_hat = y_hat.detach()   
  # compute loss solely on prediction
  test_loss.append(model.loss(y_hat,y).detach().numpy())
  y_hat = y_hat.numpy().squeeze()
  y = y.numpy().squeeze()
  subDataRes = {'y': y, 'y_hat': y_hat} 
  data_prediction.append(subDataRes)
  
fig, ax = plt.subplots()
ax.plot(test_loss)
plt.show()  

# example
for it in np.arange(1,5):
  idx = np.random.randint(0,len(test_loader))
  x_t, y_t = test_loader.dataset[idx]
  t_ = np.arange(0,150)
  fig, ax = plt.subplots()
  ax.plot(t_, data_prediction[idx]['y_hat'], 'r', t_, data_prediction[idx]['y'], 'b')
  plt.show()  

if SAVE_PREDICTIONS:
  # pickle do not seem to be a good choice for matlab
  #pickle.dump( data_prediction, open( "saved_pred.p", "wb" ) )
  #formated_data = pickle.load( open( "saved_pred.p", "rb" ) )
  scipy.io.savemat('saved_pred.mat',  {"Data": data_prediction, "Offsets":test.offsets, "Divisions":test.divisions})
  