#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Nov 20 09:36:58 2020

@author: Vincent
"""

#%%
""" IMPORTS """

import torch
import torch.nn as nn
from torch.utils.data import Dataset, DataLoader
import pytorch_lightning as pl
import numpy as np
#import pandas as pd
#import matplotlib.pyplot as plt
#from sklearn.preprocessing import MinMaxScaler
from scipy.io import loadmat
from os import path
from os import getcwd
#from tqdm import tqdm # progress bars don't seem to work with spyder

#%% 
""" MACROS & HYPERPARAMETERS """
INPUT_FILE_NAME = 'youBotRhythmicTaskSubTrajectories.mat'
BATCH_SIZE      = 26
MAX_NB_EPOCHS   = 100
BATCHES_PERCENT = 1. # This allows using only a percentage of the batches for debugging
PERCENT_TRAIN_D = 0.80
#PERCENT_VAL_D   = 0.20
IS_UNIT_TEST    = False
LOAD_TRAIN_MODEL = False
TRAIN_MODEL_VERSION = 5
LOGGING_PATH    = 'lightning_logs' 

#%% 
""" CLASSES """
  
# class LSTM(nn.Module):
#   def __init__(self, input_size=5, hidden_layer_size=100, output_size=50):
#     super().__init__()
#     self.hidden_layer_size = hidden_layer_size

#     self.lstm = nn.LSTM(input_size, hidden_layer_size, batch_first=True) # for dataLoader

#     self.linear = nn.Linear(hidden_layer_size, output_size)

#     self.hidden_cell = (torch.zeros(1,1,self.hidden_layer_size),
#                         torch.zeros(1,1,self.hidden_layer_size))
  
#   def forward(self, input_seq):
#     lstm_out, self.hidden_cell = self.lstm(input_seq.view(len(input_seq) ,1, -1), self.hidden_cell)
#     predictions = self.linear(lstm_out.view(len(input_seq), -1))
#     return predictions[-1]

class matlabDataPrePro(Dataset):
  
  def __init__(self, filepath, dataName, normalization="Magnitude", mask_size=200):
    super().__init__()
    
    self.position = list()
    self.force = list()
    self.ball = list()
    self.hand = list()
    self.isBallImpact = list()

    # TODO: improve data conversion using panda
    raw_data = loadmat(filepath)
    raw_position = raw_data[dataName][0][:]['position']
    raw_force = raw_data[dataName][0][:]['force']
    raw_ball = raw_data[dataName][0][:]['ball']
    raw_hand = raw_data[dataName][0][:]['hand']
    raw_ballImp = raw_data[dataName][0][:]['isBallImpact']
    
    self.position = np.stack(raw_position).astype("float32")
    self.force = np.stack(raw_force).astype("float32")
    self.ball = np.stack(raw_ball).astype("float32")
    
    self.chunck_size = self.position.shape[1]
    # repeat the isBallImpact array to fit the dimensions of the rest of the data
    self.isBallImpact = np.tile(np.stack(raw_ballImp), (self.chunck_size, 1)).astype("float32")
    for han_str in raw_hand:
      if han_str[0][0][0] == 'right':
        self.hand.append([[1]]*self.chunck_size)
      else:
        self.hand.append([[0]]*self.chunck_size)
    
    self.hand = np.array(self.hand).astype("float32") 
    
    # mask for the data that need to be predicted
    self.mask = np.ones((self.chunck_size,1)).astype("float32")
    self.mask[-mask_size:] = 0.
    self.mask = self.mask.astype("float32")
    
    if normalization == "Magnitude":
      # data normalization between 1 and 0 .ptp() for magnitude
      self.position = (self.position - self.position.min())/self.position.ptp()
      self.force = (self.force - self.force.min())/self.force.ptp()
      self.ball = (self.ball - self.ball.min())/self.ball.ptp()
    elif normalization == "Variance":
      # data standardization with a null mean and unitary standard deviation
      self.position = (self.position - self.position.mean())/self.position.std()
      self.force = (self.force - self.force.mean())/self.force.std()
      self.ball = (self.ball - self.ball.mean())/self.ball.std()
    else:
      print("Data was neither normalized nor standardized")
    
  def __len__(self): 
    return len(self.position)

  def __getitem__(self, idx):
    
    # subsampling position and force data for LSTM 
    subsamp_position = self.position[idx][::10]
    subsamp_force = self.force[idx][::10]
    subsamp_ball = self.ball[idx][::10]
    subsamp_isBallImp = self.isBallImpact[idx][::10]
    subsamp_hand = self.hand[idx][::10]

    # mask for the force prediction, the 50, last elements are masked
    mask = self.mask[::10]

    # input features
    x = np.concatenate([subsamp_position, np.multiply(subsamp_force,mask), 
                  subsamp_ball, subsamp_hand, subsamp_isBallImp],axis=-1) 

    # expected output 
    y = subsamp_force[:,0] # TODO improve dimensions extraction ?
    
    return x, y
  
  
class LightningLSTM(pl.LightningModule):
  
  def __init__(self, learning_rate=1e-3, input_size=5, hidden_layer_size=32, output_size=1, conv_kernel_size=3):
    super().__init__()
    
    self.learning_rate = learning_rate
    self.estimate_size = 20 # TODO: should be provided by the data?
    
    self.hidden_layer_size = hidden_layer_size
    self.conv = nn.Conv1d(input_size, hidden_layer_size, conv_kernel_size, padding=(conv_kernel_size-1)//2)
    self.activation = nn.Tanh()
    self.lstm1 = nn.LSTM(hidden_layer_size, hidden_layer_size, batch_first=True) # for dataLoader   
    self.linear = nn.Linear(hidden_layer_size, output_size)
    
    #self.loss = nn.functional.mse_loss
    self.alpha = 0.8
    
  def loss(self,y1,y2):
    
    loss1 = nn.functional.mse_loss(y1[:,1:-self.estimate_size-1],y2[:,1:-self.estimate_size-1])
    loss2 = nn.functional.mse_loss(y1[:,-self.estimate_size:-1],y2[:,-self.estimate_size:-1])
    # TODO: dig loss function for negative log likelyhood
    #nn.functional.nll_loss(input, target)
    #nn.functional.cross_entropy(input, target)
    return loss2*self.alpha + loss1*(1-self.alpha)
  
  def forward(self, x):
    # prediction/inference actions
    batch_size = x.shape[0]
    x = self.activation(self.conv(x.permute(0,2,1)).permute(0,2,1))
    hidden_cell1 = (torch.zeros(1,batch_size,self.hidden_layer_size), 
                    torch.zeros(1,batch_size,self.hidden_layer_size))
    lstm1_out, _ = self.lstm1(x, hidden_cell1)
    predictions = self.linear(lstm1_out).squeeze(axis=2) # for batch_size = 1
    # predictions = self.linear(lstm1_out).squeeze()
    return predictions

  # def backward(self, loss, optimizer, optimizer_idx):
  #   if optimizer_idx == 0:
  #       super().backward(loss, optimizer, optimizer_idx, retain_graph=True)
  #   elif optimizer_idx == 1:
  #       super().backward(loss, optimizer, optimizer_idx)

  def training_step(self, batch, batch_idx):
    
    x, y = batch    
    y_hat = self(x)
    loss = self.loss(y_hat, y)
    # The accuracy is evaluated solely on the trajectory that was masked
    acc = self.loss(y_hat[-self.estimate_size:-1], y[-self.estimate_size:-1])
    
    prog_bar = {'acc': acc}  
    # Logging to TensorBoard by default
    self.log('train_loss', loss, on_step=True, on_epoch=True, prog_bar=True)
    self.log('train_acc', acc, on_step=True, on_epoch=True, prog_bar=True)
    #print('Training Step')
    return {'loss': loss, 'progress_bar': prog_bar}

  def validation_step(self, batch, batch_idx):
    res = self.training_step(batch, batch_idx)
    # Logging to TensorBoard by default
    self.log('val_loss', res['loss'], on_step=True, on_epoch=True)
    self.log('val_acc', res['progress_bar']['acc'], on_step=True, on_epoch=True)
    return res

  # def validation_epoch_end(self, val_step_outputs):
  #   avg_val_loss = torch.tensor([x['loss'] for x in val_step_outputs]).mean()
  #   avg_val_acc = torch.tensor([x['progress_bar']['acc'] for x in val_step_outputs]).mean() 
    
  #   prog_bar = {'avg_val_acc': avg_val_acc}  
  #   self.log('avg_val_loss', avg_val_loss, on_step=False, on_epoch=True)
  #   self.log('avg_val_acc', avg_val_acc, on_step=False, on_epoch=True)
  #   return {'val_loss': avg_val_loss, 'progress_bar': prog_bar} 

  def configure_optimizers(self):
    optimizer = torch.optim.Adam(self.parameters(), lr=self.learning_rate)
    scheduler = {
         'scheduler': torch.optim.lr_scheduler.ReduceLROnPlateau(optimizer),
         'monitor': 'val_loss',
         'interval': 'epoch',
         'frequency': 1,
         'strict': True}
    
    return [optimizer], [scheduler]

  # def train_dataloader(self): 
  #   self.train, self.val = torch.utils.data.random_split(dataset, 
  
  #def val_dataloader(self):
    

class PrintCallback(pl.callbacks.Callback):
  
  def on_train_start(self, trainer, pl_module):
    print("Training is started!")
      
  def on_train_end(self, trainer, pl_module):
    print("Training is done.")  


# class DecayLearningRate(pl.callbacks.Callback):

#   def __init__(self):
#       self.old_lrs = []

#   def on_train_epoch_end(self, trainer, pl_module, outputs):
#     self.old_lrs.append(pl_module.learning_rate)
#     pl_module.learning_rate *= 0.98
#     # pl_module.configure_optimizers()

#%% 
""" FUNCTIONS """


#%%
""" MAIN """

if __name__ == "__main__":
  
  #%%
  """ DATA PROCESSING """
  
  #basepath = path.dirname(path.realpath(__file__))
  basepath = getcwd()
  filepath = path.abspath(path.join(basepath, "..", "experimental_bench_pert", 
                                    "data_2020_Nov_17", INPUT_FILE_NAME))
  
  data_set = matlabDataPrePro(filepath, 'subData', normalization="Variance")  
  
  len_data = len(data_set)  
  train_size = int(PERCENT_TRAIN_D*len_data) # /BATCH_SIZE)*BATCH_SIZE
  val_size = len_data - train_size
  #val_size = int(PERCENT_VAL_D*len(data_set)/BATCH_SIZE)*BATCH_SIZE
  #test_size  = len(data_set) - train_size - val_size
  
  train, val = torch.utils.data.random_split(data_set, [train_size, val_size])
  train_loader = DataLoader(train, batch_size=BATCH_SIZE, shuffle=True, drop_last=True)
  val_loader = DataLoader(val, batch_size=BATCH_SIZE, shuffle=False, drop_last=True)  
  #loader = DataLoader(data_set, batch_size=BATCH_SIZE, shuffle=True, drop_last=True)
  
  """ LSTM MODEL """
  
  lstm_model = LightningLSTM()
  
  if LOAD_TRAIN_MODEL:
    checkpoint_callback =  pl.callbacks.ModelCheckpoint(dirpath= path.join( 
                          LOGGING_PATH, 'version_' + str(TRAIN_MODEL_VERSION)))
    
  elif not(IS_UNIT_TEST):
    checkpoint_callback = pl.callbacks.ModelCheckpoint(
      monitor='val_loss',
      #dirpath=LOGGING_PATH + '/version_',
      filename='force_estimation-{epoch:02d}-{val_loss:.2f}',
      save_top_k=7,
      mode='min')    
  else: #TODO : Do not record anything in the case of unitary test
    checkpoint_callback = pl.callbacks.ModelCheckpoint()  
  
  early_stop_callback = pl.callbacks.early_stopping.EarlyStopping(
   monitor='val_acc',
   min_delta=0.00,
   patience=5,
   verbose=False
   #mode='max'
 )
  lr_monitor = pl.callbacks.LearningRateMonitor()
  #reconf_lr_callback = DecayLearningRate()
  
  trainer = pl.Trainer(fast_dev_run=IS_UNIT_TEST, check_val_every_n_epoch=1, 
                max_epochs=MAX_NB_EPOCHS, limit_train_batches=BATCHES_PERCENT, 
                         callbacks=[checkpoint_callback, early_stop_callback,
                                    lr_monitor], auto_lr_find=False)   
      
  trainer.fit(lstm_model, train_loader, val_loader)  
  
  #%% ipynb-py-convert lstm_trajectory_prediction_v1.py lstm_trajectory_prediction_v1.ipynb
 