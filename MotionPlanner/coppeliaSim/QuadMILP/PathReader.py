import numpy as np
import matplotlib.pyplot as plt


class Path:
    def __init__(self, index, data=None):
        self.index = index
        self.__key = "%d" % index
        if data is not None:
            self(data)

    def __call__(self, item):
        initX = [item["initial_positions"]["x"][self.index]]
        initY = [item["initial_positions"]["y"][self.index]]
        X = initX + item["paths"][self.__key]['x'] + initX
        Y = initY + item["paths"][self.__key]['y'] + initY
        # X = item["paths"][self.__key]['x']
        # Y = item["paths"][self.__key]['y']
        self.__data = np.array(list(zip(X, Y)))

    def __len__(self):
        return len(self.__data)

    def __repr__(self):
        return f"[data: {self.index}] \n {self.__data}"

    def plot(self):
        print(self.__data.shape)
        plt.plot(self.__data[:, 0], self.__data[:, 1])

    def scatter(self):
        plt.scatter(self.__data[:, 0], self.__data[:, 1])

    def scale(self, x, x_max, x_min):
        X_std = x / 100.0
        X_scaled = X_std * (x_max - x_min) + x_min
        return X_scaled

    def fit(self, x_min, x_max, y_min, y_max):
        for i, d in enumerate(self.__data):
            x, y = d
            self.__data[i][0] = self.scale(x, x_max, x_min)
            self.__data[i][1] = self.scale(y, y_max, y_min)

    def interpolate(self, velocity=0.5):
        path = self.__data.copy()
        N = len(path)
        self.__data = None
        for i in range(1, N):
            d = np.linalg.norm(path[i - 1] - path[i])
            num_points = int(d / velocity)
            x = np.linspace(path[i - 1][0], path[i][0], num_points)
            y = np.linspace(path[i - 1][1], path[i][1], num_points)
            data = np.array([x, y]).T
            if len(data):
                self.__data = data if self.__data is None else np.vstack((self.__data, data))
                # print(data.shape)

    def __getitem__(self, item):
        assert item < len(self)
        return self.__data[item]


if __name__ == '__main__':
    import json
    filename = 'area_decomposition.json'
    with open(filename) as file:
        jdata = json.load(file)
    path = Path(index=0, data=jdata)