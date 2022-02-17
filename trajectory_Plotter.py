from mpl_toolkits import mplot3d
import numpy as np
import matplotlib.pyplot as plt

lat = 69.234268
lng = 16.046560
alt = [0,0,0,0,0,0,0,-8.720512429,0,0,-8.720512429,0,-8.720512429,0,0,0,0,0,0,0,8.648173786,0,0,8.648173786,8.648173786,8.648173786,8.648173786,8.648173786,8.648173786,8.648173786,8.648173786,17.38417568,8.648173786,17.38417568,17.38417568,17.38417568,26.12798122,26.12798122,26.12798122,26.12798122,17.38417568,17.38417568,17.38417568,26.12798122,26.12798122,17.38417568,26.12798122,17.38417568,17.38417568,17.38417568,17.38417568,17.38417568,17.38417568,17.38417568,26.12798122,17.38417568,17.38417568,17.38417568,17.38417568,17.38417568,26.12798122,17.38417568,17.38417568,17.38417568,26.12798122,17.38417568,17.38417568,17.38417568,17.38417568,26.12798122,26.12798122,26.12798122,26.12798122,26.12798122,26.12798122,26.12798122,17.38417568,26.12798122,26.12798122,26.12798122,26.12798122,17.38417568,26.12798122,26.12798122,26.12798122,17.38417568,26.12798122,26.12798122,26.12798122,26.12798122,26.12798122,26.12798122,26.12798122,26.12798122,26.12798122,26.12798122,26.12798122,34.79928028,26.12798122,26.12798122,43.558668,34.79928028,34.79928028,26.12798122,26.12798122,34.79928028,34.79928028,34.79928028,34.79928028,34.79928028,43.558668,43.558668,43.558668,43.558668,43.558668,43.558668,43.558668,43.558668,43.558668,43.558668,43.558668,34.79928028,43.558668,43.558668,43.558668,43.558668,43.558668,43.558668,43.558668,43.558668,43.558668,43.558668,43.558668,43.558668,43.558668,52.32590616,52.32590616,52.32590616,52.32590616,52.32590616,52.32590616,43.558668,61.02046915,52.32590616,61.02046915,61.02046915,61.02046915,69.80338307,61.02046915,61.02046915,69.80338307,61.02046915,52.32590616,61.02046915,61.02046915,69.80338307,61.02046915,69.80338307,69.80338307,61.02046915,69.80338307,69.80338307,69.80338307,61.02046915,61.02046915,61.02046915,61.02046915,61.02046915,52.32590616,61.02046915,61.02046915,61.02046915,52.32590616,52.32590616,61.02046915,61.02046915,61.02046915,61.02046915,61.02046915,61.02046915,61.02046915,52.32590616,69.80338307,52.32590616,61.02046915,61.02046915,61.02046915,61.02046915,61.02046915,61.02046915,61.02046915,69.80338307,61.02046915,61.02046915,69.80338307,69.80338307,69.80338307,78.51350907,78.51350907,78.51350907,78.51350907,69.80338307,78.51350907,78.51350907,78.51350907,96.11874334,78.51350907,78.51350907,87.31216153,87.31216153,87.31216153,78.51350907,78.51350907,78.51350907,78.51350907,87.31216153,78.51350907,87.31216153,78.51350907,78.51350907,78.51350907,78.51350907,87.31216153,78.51350907,78.51350907,96.11874334,87.31216153,78.51350907,78.51350907,87.31216153,78.51350907,78.51350907,87.31216153,87.31216153,87.31216153,87.31216153,87.31216153,96.11874334,87.31216153,96.11874334,87.31216153,96.11874334,87.31216153,96.11874334,96.11874334,96.11874334,96.11874334,96.11874334,96.11874334,96.11874334,96.11874334,96.11874334,96.11874334,96.11874334,96.11874334,104.8523671,104.8523671,96.11874334,104.8523671,104.8523671,104.8523671,113.6747825,104.8523671,113.6747825,104.8523671,113.6747825,104.8523671,113.6747825,122.4241261,113.6747825,113.6747825,113.6747825,113.6747825,113.6747825,113.6747825,122.4241261,113.6747825,113.6747825,113.6747825,113.6747825,113.6747825,113.6747825,113.6747825,113.6747825,113.6747825,113.6747825,122.4241261,113.6747825,113.6747825,104.8523671,122.4241261,113.6747825,122.4241261,113.6747825,104.8523671,104.8523671,104.8523671,113.6747825,113.6747825,113.6747825,113.6747825,113.6747825,113.6747825,113.6747825,113.6747825,113.6747825,113.6747825,122.4241261,104.8523671,113.6747825,113.6747825,113.6747825,113.6747825,113.6747825,113.6747825,113.6747825,113.6747825,96.11874334,104.8523671,113.6747825,113.6747825,113.6747825,113.6747825,104.8523671,104.8523671,104.8523671,96.11874334,96.11874334,104.8523671,96.11874334,87.31216153,96.11874334,87.31216153,96.11874334,78.51350907,87.31216153,96.11874334,87.31216153,87.31216153,78.51350907,87.31216153,78.51350907,69.80338307,78.51350907,69.80338307,87.31216153,69.80338307,69.80338307,69.80338307,61.02046915,61.02046915,61.02046915,61.02046915,69.80338307,61.02046915,52.32590616,52.32590616,52.32590616,52.32590616,52.32590616,43.558668,43.558668,43.558668,43.558668,43.558668,43.558668,43.558668,34.79928028,34.79928028,26.12798122,34.79928028,26.12798122,26.12798122,17.38417568,17.38417568,17.38417568,26.12798122,8.648173786,17.38417568,8.648173786,0,8.648173786,8.648173786]

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

x = np.linspace(lat, lat, len(alt))
y = np.linspace(lng, lng, len(alt))

ax.plot(y, x, alt)
plt.xlim(16, 16.1)
plt.ylim(69.2, 69.3)

plt.show()