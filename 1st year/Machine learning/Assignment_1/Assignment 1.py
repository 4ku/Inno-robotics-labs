#!/usr/bin/env python
# coding: utf-8

# # Assignment 1

# ## Analyze data

from matplotlib import pyplot as plt
import pandas as pd

data = pd.read_csv('flight_delay.csv')

print("First n rows:")
print(data.head().to_string())

print()
print("Data info:")
data.info()

print()
print("Describe delay column")
print(data.describe().to_string())

#Check fo nans
print("Is there are any nulls:", data.isnull().values.any())


# ## Preprocess data

from sklearn.preprocessing import LabelEncoder
data["Scheduled arrival time"] = pd.to_datetime(data["Scheduled arrival time"])
data["Scheduled depature time"] = pd.to_datetime(data["Scheduled depature time"])
data["Duration"] = (data["Scheduled arrival time"] - data["Scheduled depature time"]).dt.seconds/60
data["arrival month"] = data["Scheduled arrival time"].dt.month
data["arrival day"] = data["Scheduled arrival time"].dt.day
data["arrival day_of_week"] = data["Scheduled arrival time"].dt.day_of_week
data["departure month"] = data["Scheduled depature time"].dt.month
data["departure day"] = data["Scheduled depature time"].dt.day
data["departure day_of_week"] = data["Scheduled depature time"].dt.day_of_week
data["departure year"] = data["Scheduled depature time"].dt.year
data.drop(columns=["Scheduled arrival time", "Scheduled depature time"], inplace=True)

print("Unique numbers of departure airports:", len(data["Depature Airport"].unique()))
print("Unique numbers of destination airports:", len(data["Destination Airport"].unique()))
le = LabelEncoder()
data["Depature Airport"] = le.fit_transform(data["Depature Airport"])
data["Destination Airport"] = le.fit_transform(data["Destination Airport"])

print("Data shape", data.shape)


# ## Some charts

# ### Duration-delay chart

plt.scatter(data["Duration"], data["Delay"])
plt.savefig('Scatter duration-delay.png')


# ### Delay histogram

data["Delay"][(data["Delay"] > 0) & (data["Delay"] < 50)].hist(bins=50)
plt.savefig('Delay under 50.png')

data["Delay"][(data["Delay"] > 50) & (data["Delay"] < 300)].hist(bins=50)
plt.savefig('Delay over 50 and under 300.png')

data["Delay"][(data["Delay"] > 300)].hist(bins=50)
plt.savefig('Delay over 300.png')


# ### Normalize duration column

from sklearn.preprocessing import RobustScaler
rs = RobustScaler()

data["Duration"] = rs.fit_transform(data["Duration"].values[:, None])


# ## Split data to train and test

train_data = data[data['departure year'] < 2018]
test_data = data[data['departure year'] >= 2018]
trainY = train_data["Delay"]
trainX= train_data.drop(columns=["Delay", 'departure year'])
testY = test_data["Delay"]
testX= test_data.drop(columns=["Delay", 'departure year'])


# ## Find outliers and remove it

import numpy as np
from sklearn.ensemble import IsolationForest

train_data = train_data.drop(columns=['departure year'])
oulier_detector = IsolationForest(n_estimators=200, contamination=0.05, n_jobs=-1)
outliers = oulier_detector.fit_predict(train_data)
print("Unique numbers for outliers:", np.unique(outliers))
print(f"Number of outliers: {(outliers == -1).sum()} of {len(outliers)}")


# ### Outliers chart

plt.scatter(train_data["Duration"][outliers == -1], train_data["Delay"][outliers == -1], c='r')
plt.scatter(train_data["Duration"][outliers == 1], train_data["Delay"][outliers == 1], c='b')
plt.savefig('Outliers.png')

trainX = trainX[outliers == 1]
trainY = trainY[outliers == 1]


# ## Machine learning models

# Import libraries
import numpy as np
from sklearn.linear_model import LinearRegression, Ridge, Lasso
from sklearn.preprocessing import PolynomialFeatures

from sklearn.metrics import mean_squared_error, mean_absolute_error
from sklearn.model_selection import GridSearchCV


# ### Linear regression

LR = LinearRegression(n_jobs=-1)
LR.fit(trainX, trainY)
pred = LR.predict(testX)
print("Error for linear regression (MSE):", mean_squared_error(testY, pred))
print("Error for linear regression (MAE):", mean_absolute_error(testY, pred))
pred_train = LR.predict(trainX)
print("Error for linear regression (MSE) on train dataset:", mean_squared_error(trainY, pred_train))
print("Error for linear regression (MAE) on train dataset:", mean_absolute_error(trainY, pred_train))


# ### Ridge regression

ridge = Ridge()
gridParams = {"alpha": np.linspace(0.1, 4, 5)}
grid = GridSearchCV(ridge, gridParams, scoring='neg_mean_squared_error',
                    verbose=1, n_jobs=-1, cv=5)
grid.fit(trainX, trainY)
print("Best params:", grid.best_params_)
print("Best score:", grid.best_score_)
print("Error for ridge regression (MSE):", mean_squared_error(testY, grid.predict(testX)))
print("Error for ridge regression (MAE):", mean_absolute_error(testY, grid.predict(testX)))
pred_train = grid.predict(trainX)
print(f"MSE for lasso regression on train dataset:", mean_squared_error(trainY, pred_train))
print(f"MAE for lasso regression on train dataset:", mean_absolute_error(trainY, pred_train))


# ### Lasso regression

lasso = Lasso()
gridParams = {"alpha": np.linspace(0.1, 4, 5)}
grid = GridSearchCV(lasso, gridParams, scoring='neg_mean_squared_error',
                    verbose=1, n_jobs=-1, cv=5)
grid.fit(trainX, trainY)           
print("Best params:", grid.best_params_)
print("Best score:", grid.best_score_)
print("Error for lasso regression (MSE):", mean_squared_error(testY, grid.predict(testX)))
print("Error for lasso regression (MAE):", mean_absolute_error(testY, grid.predict(testX)))
pred_train = grid.predict(trainX)
print(f"MSE for lasso regression on train dataset:", mean_squared_error(trainY, pred_train))
print(f"MAE for lasso regression on train dataset:", mean_absolute_error(trainY, pred_train))


# ### Polynomial regression

degrees = [2, 3, 4]
for degree in degrees:
    poly = PolynomialFeatures(degree)
    poly.fit(trainX)
    polyXtrain = poly.transform(trainX)
    polyXtest = poly.transform(testX)
    lr = LinearRegression()
    lr.fit(polyXtrain, trainY)
    pred = lr.predict(polyXtest)
    print(f"MSE for linear regression with degree {degree} :", mean_squared_error(testY, pred))
    print(f"MAE for linear regression with degree {degree} :", mean_absolute_error(testY, pred))
    pred_train = lr.predict(polyXtrain)
    print(f"MSE for linear regression with degree {degree} on train dataset:", mean_squared_error(trainY, pred_train))
    print(f"MAE for linear regression with degree {degree} on train dataset:", mean_absolute_error(trainY, pred_train))
