import pandas as pd
from sklearn import tree

df = pd.read_csv('C:\\Users\\ajaya\\projects\\Truck Platooning\\Train Data\\Truck-Data.csv')
inputs = df.drop('master',axis='columns')
target = df['master']

model = tree.DecisionTreeClassifier()
model.fit(inputs, target)
m_score = model.score(inputs,target)
print('score:', m_score)
prediction = model.predict([[33,35,65,15,2022]])
print('The truck can be master =', prediction)
