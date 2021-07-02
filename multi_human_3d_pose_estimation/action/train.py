# import pandas as pd
# from enum import Enum
# import numpy as np
# from sklearn.preprocessing import LabelEncoder
# from sklearn.model_selection import train_test_split
# from keras.utils import np_utils
# from keras.models import Sequential
# from keras.layers import Dense, Dropout
# from keras.layers.normalization import BatchNormalization
# from keras.optimizers import Adam
# from keras.models import load_model

# import matplotlib.pyplot as plt
# from keras.callbacks import Callback
# import itertools
# from sklearn.metrics import confusion_matrix


# class Actions(Enum):
#     # framewise_recognition.h5
#     # squat = 0
#     # stand = 1
#     # walk = 2
#     # wave = 3

#     # framewise_recognition_under_scene.h5
#     stand = 0
#     walk = 1
#     operate = 2
#     fall_down = 3
#     # run = 4


# # Callback class to visialize training progress
# class LossHistory(Callback):
#     def on_train_begin(self, logs={}):
#         self.losses = {'batch':[], 'epoch':[]}
#         self.accuracy = {'batch':[], 'epoch':[]}
#         self.val_loss = {'batch':[], 'epoch':[]}
#         self.val_acc = {'batch':[], 'epoch':[]}

#     def on_batch_end(self, batch, logs={}):
#         self.losses['batch'].append(logs.get('loss'))
#         self.accuracy['batch'].append(logs.get('acc'))
#         self.val_loss['batch'].append(logs.get('val_loss'))
#         self.val_acc['batch'].append(logs.get('val_acc'))

#     def on_epoch_end(self, batch, logs={}):
#         self.losses['epoch'].append(logs.get('loss'))
#         self.accuracy['epoch'].append(logs.get('acc'))
#         self.val_loss['epoch'].append(logs.get('val_loss'))
#         self.val_acc['epoch'].append(logs.get('val_acc'))

#     def loss_plot(self, loss_type):
#         iters = range(len(self.losses[loss_type]))
#         plt.figure()
#         # acc
#         plt.plot(iters, self.accuracy[loss_type], 'r', label='train acc')
#         # loss
#         plt.plot(iters, self.losses[loss_type], 'g', label='train loss')
#         if loss_type == 'epoch':
#             # val_acc
#             plt.plot(iters, self.val_acc[loss_type], 'b', label='val acc')
#             # val_loss
#             plt.plot(iters, self.val_loss[loss_type], 'k', label='val loss')
#         plt.grid(True)
#         plt.xlabel(loss_type)
#         plt.ylabel('acc-loss')
#         plt.legend(loc="upper right")
#         plt.show()


# def plot_confusion_matrix(cm, classes,
#                           normalize=False,
#                           title='Confusion matrix',
#                           cmap=plt.cm.Blues):
#     """
#     This function prints and plots the confusion matrix.
#     Normalization can be applied by setting `normalize=True`.
#     """
#     if normalize:
#         cm = cm.astype('float') / cm.sum(axis=1)[:, np.newaxis]
#         print("Normalized confusion matrix")
#     else:
#         print('Confusion matrix, without normalization')

#     print(cm)

#     plt.imshow(cm, interpolation='nearest', cmap=cmap)
#     plt.title(title)
#     plt.colorbar()
#     tick_marks = np.arange(len(classes))
#     plt.xticks(tick_marks, classes, rotation=45)
#     plt.yticks(tick_marks, classes)

#     fmt = '.2f' if normalize else 'd'
#     thresh = cm.max() / 2.
#     for i, j in itertools.product(range(cm.shape[0]), range(cm.shape[1])):
#         plt.text(j, i, format(cm[i, j], fmt),
#                  horizontalalignment="center",
#                  color="white" if cm[i, j] > thresh else "black")

#     plt.tight_layout()
#     plt.ylabel('True label')
#     plt.xlabel('Predicted label')


# # load data
# raw_data = pd.read_csv('data_with_scene.csv', header=0)
# dataset = raw_data.values
# # X = dataset[:, 0:36].astype(float)
# # Y = dataset[:, 36]
# X = dataset[0:3289, 0:36].astype(float)  # 忽略run数据
# Y = dataset[0:3289, 36]

# # 将类别编码为数字
# # encoder = LabelEncoder()
# # encoder_Y = encoder.fit_transform(Y)
# # print(encoder_Y[0], encoder_Y[900], encoder_Y[1800], encoder_Y[2700])
# # encoder_Y = [0]*744 + [1]*722 + [2]*815 + [3]*1008 + [4]*811
# encoder_Y = [0]*744 + [1]*722 + [2]*815 + [3]*1008
# # one hot 编码
# dummy_Y = np_utils.to_categorical(encoder_Y)

# # train test split
# X_train, X_test, Y_train, Y_test = train_test_split(X, dummy_Y, test_size=0.1, random_state=9)

# # build keras model
# model = Sequential()
# model.add(Dense(units=128, activation='relu'))
# model.add(BatchNormalization())
# model.add(Dense(units=64, activation='relu'))
# model.add(BatchNormalization())
# model.add(Dense(units=16, activation='relu'))
# model.add(BatchNormalization())
# model.add(Dense(units=4, activation='softmax'))  # units = nums of classes

# # training
# his = LossHistory()
# model.compile(loss='categorical_crossentropy', optimizer=Adam(0.0001), metrics=['accuracy'])
# model.fit(X_train, Y_train, batch_size=32, epochs=20, verbose=1, validation_data=(X_test, Y_test), callbacks=[his])
# model.summary()
# his.loss_plot('epoch')
# # model.save('framewise_recognition.h5')

# # evaluate and draw confusion matrix
# print('Test:')
# score, accuracy = model.evaluate(X_test,Y_test,batch_size=32)
# print('Test Score:{:.3}'.format(score))
# print('Test accuracy:{:.3}'.format(accuracy))
# # confusion matrix
# Y_pred = model.predict(X_test)
# cfm = confusion_matrix(np.argmax(Y_test,axis=1), np.argmax(Y_pred, axis=1))
# np.set_printoptions(precision=2)

# plt.figure()
# class_names = ['squat', 'stand', 'walk', 'wave']
# plot_confusion_matrix(cfm, classes=class_names, title='Confusion Matrix')
# plt.show()

# # # test
# # model = load_model('framewise_recognition.h5')
# #
# # test_input = [0.43, 0.46, 0.43, 0.52, 0.4, 0.52, 0.39, 0.61, 0.4,
# #               0.67, 0.46, 0.52, 0.46, 0.61, 0.46, 0.67, 0.42, 0.67,
# #               0.42, 0.81, 0.43, 0.91, 0.45, 0.67, 0.45, 0.81, 0.45,
# #               0.91, 0.42, 0.44, 0.43, 0.44, 0.42, 0.46, 0.44, 0.46]
# # test_np = np.array(test_input)
# # test_np = test_np.reshape(-1, 36)
# #
# # test_np = np.array(X[1033]).reshape(-1, 36)
# # if test_np.size > 0:
# #     pred = np.argmax(model.predict(test_np))
# #     init_label = Actions(pred).name
# #     print(init_label)


import torch
import torch.nn as nn
import torch.optim as optim
import argparse
import pandas as pd
import numpy
from torch.utils.data import DataLoader,Dataset,random_split,Subset
from torchsummary import summary
from IPython import embed

import os

# action_label = {'kick':0,'punch':1,'squat':2,'stand':3,'wave':4,
#                 'youchayao':5
# }

#训练次数
epchos = 100
batch_size = 512

#路径
CURRENT_PATH = os.path.dirname(os.path.abspath(__file__))

#分割数据集
Shape = [10334,37]
dataset_len = Shape[0]
train_size = int(0.8*dataset_len)
test_size = int(0.2*dataset_len)


class Net(nn.Module):
    def __init__(self):
        super(Net,self).__init__()
        self.block = nn.Sequential(
                nn.Linear(36, 300),
                nn.ReLU(),
                nn.Linear(300, 200),
                # nn.Dropout(0.5),
                nn.ReLU(),
                nn.Linear(200, 100),
                nn.ReLU(),
                nn.Linear(100, 50),
                # nn.Dropout(0.9),
                nn.ReLU(),
                # nn.Linear(200, 150),
                nn.Linear(50, 5)
        )
        self.fc = nn.Linear(5, 36)
        self.ac1 = nn.ReLU()
        self.ac2 = nn.Sigmoid()
        self.drop = nn.Dropout(0.5)
        self.soft = nn.Softmax()

    def forward(self,x):
        out = self.block(x)
        out = self.fc(out)
        out = self.block(out)
        # out = self.soft(out)　　#loss　function为交叉熵时，不用再使用这个softmax层了
        return out

# net = Net().to('cuda')
# summary(net,(1,36))
# embed()
# loss = nn.CrossEntropyLoss()
# net = Net()
# optimizer  = optim.SGD(net.parameters(),lr=0.001,momentum=0.5)

class Data(Dataset):
    def __init__(self,data,flag='train'):
        # super(Dataset,self).__init__()
        self.data = data
        self.data_info = self.getInfo(self.data,flag)


    @staticmethod
    def getInfo(data,flag):
        if flag == 'train':
            data_info = [[] for i in range(train_size)]   #创建二维数组
            size = train_size
        else:
            data_info = [[] for i in range(test_size)]
            size = test_size
        # print(len(data_info))
        # print(data[0][0:37])
        for idx in range(size):
            data_info[idx] = (data[idx][0:36],int(data[idx][36]))
        return data_info


    def __getitem__(self, index):
        coordi,lable = self.data_info[index]
        return coordi,lable

    def __len__(self):
        return len(self.data_info)



if __name__ == '__main__':
    CURRENT_PATH = os.path.dirname(os.path.abspath(__file__)) + '/'

    key_word = '--dataset'
    paser = argparse.ArgumentParser()
    paser.add_argument(key_word,required=False,help = 'display the dataset',default='../data/train_data.csv')

    opt = paser.parse_args().dataset

    #加载数据集(关节点的数据集)
    try:
        raw_data = pd.read_csv(opt,header=0)
    except:
        print("dataset not exist")

    dataset = raw_data.values
    # print(dataset)

    # X = dataset[:,0:36].astype(float)
    # Y = dataset[:,36]
    # print(Y)

    #将动作标签进行转换
    # for i in range(len(Y)):
    #     # # print(key)
    #     if Y[i] in action_label.keys():
    #         Y[i] = action_label[Y[i]]
    # print(Y.astype)   #numpy
    # Y = Y.astype(float)  #强制转换类型，满足torch支持的类型

    #对数据进行优化
    # X_pp = []
    # total = []
    # for i in range(len(X)):
    #     X_pp.append(dpp.pose_normalization(X[i]))
    # total = X_pp

    # total = torch.Tensor(X_pp)
    # print(X_pp.shape)
    # Y_pp = torch.from_numpy(Y)

    # for i in range(len(X_pp)):
    #     total[i].append(Y[i])
    # # print(total)
    # total = torch.Tensor(total)    #torch.Size([4416, 27])
    # Shape = total.shape            #得到shape

    total = torch.Tensor(dataset)
    train_set = Subset(total,range(train_size))  #训练集
    test_set = Subset(total,range(train_size,dataset_len)) #测试集
    #数据处理完成＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝

    dataset_1 = Data(train_set,'train')
    train_loader = DataLoader(dataset=dataset_1,batch_size = batch_size,shuffle = True)
    # print(len(dataset_1))/
    loss_C = nn.CrossEntropyLoss()
    net = Net()

    # for name, parameters in net.named_parameters():
    #     print(name, ':', parameters.size())

    optimizer = optim.Adagrad(net.parameters(), lr=0.003)    #调整一下学习率
    #训练
    loss = 0
    total_loss = 0
    correct_num = 0
    for epcho in range(epchos):
        epo_loss = 0   #一个epoch中的loss
        for zips in train_loader:
            train_data = zips[0]
            # print(len(train_data))
            label = zips[1]
            optimizer.zero_grad()
            output = net(train_data)
            max_label = output.argmax(dim = 1)
            # print("max_label:",max_label.data)
            correct_num += max_label.eq(label.data).float().sum().item()
            # print(correct_num)
            # print("label.data",label.data)
            loss = loss_C(output,label)
            total_loss += loss
            epo_loss += loss
            # print(loss)
            loss.backward()
            optimizer.step()


        if epcho % 10 == 0 and epcho != 0:
            # print(output.shape)
            # print(label.shape)
            print("train_epcho: {} -- loss:{:0.3f} -- total_loss:{:0.3f}".format(epcho,epo_loss,total_loss))
            # print("train_epcho: {}   Avg_loss: {:.6f}".format(epcho,total_loss / batch_size))
            print('===============================================================')

    #
    # print("total_loss: {:.6f}".format(total_loss))
    # print('==========================')
    # print("avgLoss: {:.6f}".format(total_loss / len(dataset_1)))
    # print(correct_num)
    print("Acc: {:0.2f}%".format((correct_num / (train_size * epchos) * 100)))

    state_dict = net.state_dict()
    # print(state_dict['block.0.bias'])
    torch.save(state_dict,'../model/action.pkl')

    ####test
    state_dict_test = torch.load('../model/action.pkl')
    net = Net()
    net.load_state_dict(state_dict_test)

    net.eval()
    dataset_2 = Data(test_set, 'test')   #制作数据
    test_loader = DataLoader(dataset=dataset_2,batch_size=64,shuffle=False) #取出数据
    #begin to predict
    correct_num = 0
    for zips in test_loader:
        test_data = zips[0]
        # print(zips[0])
        test_label = zips[1]
        test_output = net(test_data)
        max_label = test_output.argmax(dim=1)
        # print(test_output)
        correct_num += max_label.eq(test_label.data).float().sum().item()

    print("the test-acc is {:0.2f}%".format((correct_num / test_size  * 100)))

