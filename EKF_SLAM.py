import numpy as np


def motion_update(mu_t_1, sigma_t_1, u_t, dt, N):
    F_x = np.hstack((np.eye(3), np.zeros((3, 2*N))))  # line: 2
    F_x_T = F_x.transpose()

    # test
    print('\nF_x >>')
    print(F_x)
    # test/

    '''
    v_t >> translational velocity, u_t[0]
    w_t >> rotatinal velocity, u_t[1]
    '''
    v_t = u_t[0]
    w_t = u_t[1]
    mu_t_1_theta = mu_t_1[2, 0]
    temp_matrix = np.array([[-(v_t/w_t)*np.sin(mu_t_1_theta)+(v_t/w_t)*np.sin(mu_t_1_theta+w_t*dt)],
                            [(v_t/w_t)*np.cos(mu_t_1_theta)-(v_t/w_t)*np.cos(mu_t_1_theta+w_t*dt)],
                            [w_t*dt]])

    mu_t_bar = mu_t_1 + np.dot(F_x_T, temp_matrix)  # line: 3

    temp_matrix = np.array([[0, 0, (v_t/w_t)*np.cos(mu_t_1_theta)-(v_t/w_t)*np.cos(mu_t_1_theta+w_t*dt)],
                            [0, 0, (v_t/w_t)*np.sin(mu_t_1_theta)-(v_t/w_t)*np.sin(mu_t_1_theta+w_t*dt)],
                            [0, 0, 0]])

    G_t = np.eye(2*N + 3) + np.dot(np.dot(F_x_T, temp_matrix), F_x)  # line: 4

    # test
    print('\nG_t >>')
    print(G_t)
    # test/

    G_t_T = G_t.transpose()

    R_t = np.array([[v_t + 0.5 * np.random.randn() + 0, 0, 0],
                    [0, w_t + 0.5 * np.random.randn() + 0, 0],
                    [0, 0, mu_t_1_theta + 0.5 * np.random.randn() + 0]])  # tuning parameter

    # test
    print('\nR_t >>')
    print(R_t)
    # test/

    sigma_t_bar = np.dot(np.dot(G_t, sigma_t_1), G_t_T) + np.dot(np.dot(F_x_T, R_t), F_x)  # line: 5

    # print(F_x)  # test
    print('\nmu_t_bar >>')  # test
    print(mu_t_bar)  # test

    # print(G_t)  # test
    print('\nsigma_t_bar >>')  # test
    print(sigma_t_bar)  # test

    return mu_t_bar, sigma_t_bar


def measurement_updatae(mu_t_bar, sigma_t_bar, z_t, N):
    sigma_r = 0 + 0.5 * np.random.randn() + 0  # tuning parameter
    sigma_pi = 0 + 0.5 * np.random.randn() + 0  # tuning parameter
    sigma_s = 0 + 0.5 * np.random.randn() + 0  # tuning parameter
    Q_t = np.array([[sigma_r, 0, 0],
                    [0, sigma_pi, 0],
                    [0, 0, sigma_s]])  # line: 6  # tuning parameter

    # test
    print('\nQ_t >>')
    print(Q_t)
    # test/

    z_t_T = z_t.T

    # test
    print('\nz_t >>')
    print(z_t)
    # test/

    for [r_t, pi_t, s_t] in z_t_T:  # line: 7 ~ 18  # dist, angle, map_id
        j = int(s_t)  # line: 8

        order = ['st', 'nd']
        if sigma_t_bar[2*j-1][2*j-1] >= 1e6 and sigma_t_bar[2*j+1][2*j+1] >= 1e6:  # line: 9 ~ 11
            print('\n%d%s landmark is never seen before\n' % (j, order[j-1]))  # test

            mu_t_bar[2 * j - 1][0] = mu_t_bar[0][0] + r_t * np.cos(pi_t + mu_t_bar[2][0])
            mu_t_bar[2 * j + 1][0] = mu_t_bar[1][0] + r_t * np.sin(pi_t + mu_t_bar[2][0])  # line: 10

        delta_x = mu_t_bar[2 * j - 1][0] - mu_t_bar[0][0]
        delta_y = mu_t_bar[2 * j + 1][0] - mu_t_bar[1][0]
        delta = np.array([[delta_x],
                          [delta_y]])  # line: 12

        # test
        print('\ndelta >>')
        print(delta)
        # test/

        q = np.dot(np.transpose(delta), delta)  # line: 13
        root_q = np.squeeze(np.sqrt(q))

        # test
        print('\nq >>')
        print(q)
        # test/

        z_t_hat = np.array([[root_q],
                            [np.arctan2(delta_y, delta_x) - mu_t_bar[2, 0]],
                            [j]])  # line: 14

        # test
        print('\nz_t_hat >>')
        print(z_t_hat)
        # test/

        F_x_j = np.zeros((5, 2*N+3))
        F_x_j[:3, :3] = np.eye(3)
        F_x_j[3, j + 2] = 1
        F_x_j[4, j + 3] = 1  # line: 15
        F_x_j = np.vstack((F_x_j, [0, 0, 0, 0, 0, 0, 1]))  # line: 15

        # test
        print('\nF_x_j >>')
        print(F_x_j)
        # test/

        # test.
        H_t = (1/q) * np.dot(np.array([[root_q*delta_x, -root_q*delta_y, 0, -root_q*delta_x, root_q*delta_y, 0],
                                       [delta_y, delta_x, -1, -delta_y, -delta_x, 0],
                                       [0, 0, 0, 0, 0, 1]]), F_x_j)  # line:16

        # test
        print('\nH_t >>')
        print(H_t)
        # test/

        temp_matrix_1 = np.dot(sigma_t_bar, np.transpose(H_t))
        temp_matrix_2 = np.dot(np.dot(H_t, sigma_t_bar), np.transpose(H_t)) + Q_t

        # temp_matrix_2_inverse = np.linalg.inv(temp_matrix_2)
        temp_matrix_2_inverse = 1/temp_matrix_2

        K_t = np.dot(temp_matrix_1, temp_matrix_2_inverse)  # line: 17

        # test
        print('\nK_t >>')
        print(K_t)
        # test/

    mu_t = mu_t_bar + np.sum(np.dot(K_t, (z_t-z_t_hat)))  # line:19
    sigma_t = np.dot((np.eye(2*N + 3) - np.sum(np.dot(K_t, H_t))), sigma_t_bar)  # line: 20

    return mu_t, sigma_t


def EKF_SLAM(mu_t_1, sigma_t_1, u_t, z_t, c_t, dt, N):
    mu_t_bar, sigma_t_bar = motion_update(mu_t_1, sigma_t_1, u_t, dt, N)
    mu_t, sigma_t = measurement_updatae(mu_t_bar, sigma_t_bar, z_t, N)

    return mu_t, sigma_t  # line: 21
