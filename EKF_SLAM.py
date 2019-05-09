import numpy as np


def initialization(N, car_state, target_info, vehicle_pose):
    print('Initialization for EKF SLAM')  # test
    # initialization
    inf = 1e6

    mu_0 = np.transpose([np.zeros(2 * N + 3)])

    # mu_0[0, 0] = 0
    # mu_0[1, 0] = 0
    # mu_0[2, 0] = 0
    # mu_0[3, 0] = 0
    # mu_0[4, 0] = 0
    # mu_0[5, 0] = 0
    # mu_0[6, 0] = 0
    # # mu_0[0, 0] = car_state.PosX
    # # mu_0[1, 0] = car_state.PosY
    # mu_0[0, 0] = target_info.PosX
    # mu_0[1, 0] = target_info.PosY
    # # mu_0[2, 0] = np.deg2rad(car_state.heading)  # car_state.heading * np.pi / 180  # car_state.heading
    # mu_0[2, 0] = np.deg2rad(target_info.heading)
    # mu_0[3, 0] = target_info.targetPosX1
    # mu_0[4, 0] = target_info.targetPosY1
    # mu_0[5, 0] = target_info.targetPosX2
    # mu_0[6, 0] = target_info.targetPosY2
    # mu_0[0, 0] = vehicle_pose[0]
    # mu_0[1, 0] = vehicle_pose[1]
    # mu_0[2, 0] = np.deg2rad(car_state.heading)  # car_state.heading
    # mu_0[3, 0] = target1_[0]
    # mu_0[4, 0] = target1_[1]
    # mu_0[5, 0] = target2_[0]
    # mu_0[6, 0] = target2_[1]
    # test
    mu_0[0, 0] = -1 * (car_state.PosX + np.cos(np.deg2rad(car_state.heading))*(2.7/2))
    mu_0[1, 0] = car_state.PosY + np.sin(np.deg2rad(car_state.heading))*(2.7/2)
    mu_0[2, 0] = np.deg2rad(car_state.heading)
    # mu_0[3, 0] = -1 * (mu_0[0, 0] - target_info.targetPosY1)
    # mu_0[4, 0] = mu_0[1, 0] + target_info.targetPosX1
    # mu_0[5, 0] = target_info.targetPosY2 + mu_0[0, 0]
    # mu_0[6, 0] = target_info.targetPosX2 - mu_0[1, 0]
    # mu_0[3, 0] = target_info.targetPosY1 - mu_0[0, 0]
    # mu_0[4, 0] = target_info.targetPosX1 - mu_0[1, 0]
    # mu_0[5, 0] = target_info.targetPosY2 - mu_0[0, 0]
    # mu_0[6, 0] = target_info.targetPosX2 - mu_0[1, 0]

    # 0508
    # mu_0[3, 0] = np.sqrt(pow(mu_0[0, 0] - target_info.targetPosY1, 2) + pow(mu_0[1, 0] - target_info.targetPosX1, 2)) * np.cos(mu_0[2, 0])
    # mu_0[4, 0] = np.sqrt(pow(mu_0[0, 0] - target_info.targetPosY1, 2) + pow(mu_0[1, 0] - target_info.targetPosX1, 2)) * np.sin(mu_0[2, 0])
    # mu_0[5, 0] = np.sqrt(pow(mu_0[0, 0] - target_info.targetPosY2, 2) + pow(mu_0[1, 0] - target_info.targetPosX2, 2)) * np.cos(mu_0[2, 0])
    # mu_0[6, 0] = np.sqrt(pow(mu_0[0, 0] - target_info.targetPosY2, 2) + pow(mu_0[1, 0] - target_info.targetPosX2, 2)) * np.sin(mu_0[2, 0])

    mu_0[3, 0] = np.sqrt(pow(mu_0[0, 0] - target_info.targetPosY1, 2) + pow(mu_0[1, 0] - target_info.targetPosX1, 2)) * np.cos(np.arctan2(target_info.targetPosX1 - mu_0[1, 0], target_info.targetPosY1 - mu_0[0, 0]) - mu_0[2, 0])
    mu_0[4, 0] = np.sqrt(pow(mu_0[0, 0] - target_info.targetPosY1, 2) + pow(mu_0[1, 0] - target_info.targetPosX1, 2)) * np.sin(np.arctan2(target_info.targetPosX1 - mu_0[1, 0], target_info.targetPosY1 - mu_0[0, 0]) - mu_0[2, 0])
    mu_0[5, 0] = np.sqrt(pow(mu_0[0, 0] - target_info.targetPosY2, 2) + pow(mu_0[1, 0] - target_info.targetPosX2, 2)) * np.cos(np.arctan2(target_info.targetPosX2 - mu_0[1, 0], target_info.targetPosY2 - mu_0[0, 0]) - mu_0[2, 0])
    mu_0[6, 0] = np.sqrt(pow(mu_0[0, 0] - target_info.targetPosY2, 2) + pow(mu_0[1, 0] - target_info.targetPosX2, 2)) * np.sin(np.arctan2(target_info.targetPosX2 - mu_0[1, 0], target_info.targetPosY2 - mu_0[0, 0]) - mu_0[2, 0])
    # 0508/
    # test/

    # sigma_0 = inf * np.ones((2 * N + 3, 2 * N + 3))
    sigma_0 = inf * np.eye(2 * N + 3)  # check
    sigma_0[:3, :3] = np.zeros((3, 3))

    global mu_t_1
    global sigma_t_1
    mu_t_1 = mu_0
    sigma_t_1 = sigma_0
    # initialization/

    # test
    print('\nmu_0 >>')
    print(mu_0)
    print('\nsigma_0 >>')
    print(sigma_0)
    # test/

    return mu_t_1, sigma_t_1


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
    velocity_model = np.array([[-(v_t/w_t)*np.sin(mu_t_1_theta)+(v_t/w_t)*np.sin(mu_t_1_theta+w_t*dt)],
                               [(v_t/w_t)*np.cos(mu_t_1_theta)-(v_t/w_t)*np.cos(mu_t_1_theta+w_t*dt)],
                               [w_t*dt]])  # velocity_model: standard noise-free velocity model
    # velocity_model = np.array([[-(v_t/w_t)*np.cos(mu_t_1_theta)+(v_t/w_t)*np.cos(mu_t_1_theta+w_t*dt)],
    #                            [-(v_t/w_t)*np.sin(mu_t_1_theta)+(v_t/w_t)*np.sin(mu_t_1_theta+w_t*dt)],
    #                            [w_t*dt]])  # velocity_model: standard noise-free velocity model
    # 0508
    # velocity_model = np.array([[-(v_t/w_t)*np.sin(mu_t_1_theta)+(v_t/w_t)*np.sin(mu_t_1_theta+w_t*dt)],
    #                            [-(v_t/w_t)*np.cos(mu_t_1_theta)-(v_t/w_t)*np.cos(mu_t_1_theta+w_t*dt)],
    #                            [w_t*dt]])  # velocity_model: standard noise-free velocity model
    # 0508/


    mu_t_bar = mu_t_1 + np.dot(F_x_T, velocity_model)  # line: 3

    # g_t = np.array([[0, 0, (v_t/w_t)*np.cos(mu_t_1_theta)-(v_t/w_t)*np.cos(mu_t_1_theta+w_t*dt)],
    #                 [0, 0, (v_t/w_t)*np.sin(mu_t_1_theta)-(v_t/w_t)*np.sin(mu_t_1_theta+w_t*dt)],
    #                 [0, 0, 0]])  # g: motion function
    g_t = np.array([[0, 0, -(v_t/w_t)*np.sin(mu_t_1_theta)+(v_t/w_t)*np.sin(mu_t_1_theta+w_t*dt)],
                    [0, 0, (v_t/w_t)*np.cos(mu_t_1_theta)+(v_t/w_t)*np.cos(mu_t_1_theta+w_t*dt)],
                    [0, 0, 0]])  # g: motion function

    # 0508
    # g_t = np.array([[0, 0, -(v_t/w_t)*np.cos(mu_t_1_theta)+(v_t/w_t)*np.cos(mu_t_1_theta+w_t*dt)],
    #                 [0, 0, (v_t/w_t)*np.sin(mu_t_1_theta)+(v_t/w_t)*np.sin(mu_t_1_theta+w_t*dt)],
    #                 [0, 0, 0]])  # g: motion function
    # 0508/

    G_t = np.eye(2*N + 3) + np.dot(np.dot(F_x_T, g_t), F_x)  # line: 4  # Gt: Jacobian of velocity model

    # 0508
    # G_t = np.eye(2*N + 3)
    # G_t[:3, :3] = g_t  # line: 4  # Gt: Jacobian of velocity model
    # 0508/

    # test
    print('\nG_t >>')
    print(G_t)
    # test/

    G_t_T = G_t.transpose()

    # check
    # R_t = np.array([[v_t + 0.5 * np.random.randn() + 0, 0, 0],
    #                 [0, w_t + 0.5 * np.random.randn() + 0, 0],
    #                 [0, 0, mu_t_1_theta + 0.5 * np.random.randn() + 0]])  # tuning parameter
    R_t = np.array([[0.5, 0, 0],
                    [0, 0.05, 0],
                    [0, 0, 0.05]])  # tuning parameter
    # check/

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


def measurement_update(mu_t_bar, sigma_t_bar, z_t, N):
    # check
    # sigma_r = 0 + 0.5 * np.random.randn() + 0  # tuning parameter
    # sigma_pi = 0 + 0.5 * np.random.randn() + 0  # tuning parameter
    # sigma_s = 0 + 0.5 * np.random.randn() + 0  # tuning parameter
    sigma_r = 0.01  # tuning parameter
    sigma_pi = 0.01  # tuning parameter
    sigma_s = 0.01  # tuning parameter
    # Q_t = np.array([[sigma_r, 0, 0],
    #                 [0, sigma_pi, 0],
    #                 [0, 0, sigma_s]])  # line: 6  # tuning parameter
    Q_t = np.array([[sigma_r, 0],
                    [0, sigma_pi]])  # line: 6  # tuning parameter
    # check/

    # test
    print('\nQ_t >>')
    print(Q_t)
    # test/

    z_t_T = z_t.T

    # test
    print('\nz_t >>')
    print(z_t)
    # test/

    mu_dif = 0
    sig_dif = 0
    for [r_t, pi_t, s_t] in z_t_T:  # line: 7 ~ 18  # dist, angle, map_id
        j = int(s_t)  # line: 8

        order = ['st', 'nd']
        if sigma_t_bar[2*j+1][2*j+1] >= 1e50 and sigma_t_bar[2*j+2][2*j+2] >= 1e50:  # line: 9 ~ 11
            print('\n%d%s landmark has never been observed before\n' % (j, order[j-1]))  # test

            mu_t_bar[2 * j + 1][0] = mu_t_bar[0][0] + r_t * np.cos(pi_t + mu_t_bar[2][0])
            mu_t_bar[2 * j + 2][0] = mu_t_bar[1][0] + r_t * np.sin(pi_t + mu_t_bar[2][0])  # line: 10

        delta_x = mu_t_bar[2 * j + 1][0] - mu_t_bar[0][0]
        delta_y = mu_t_bar[2 * j + 2][0] - mu_t_bar[1][0]
        delta = np.array([[delta_x],
                          [delta_y]])  # line: 12

        # test
        print('\ndelta >>')
        print(delta)
        # test/

        q = np.dot(np.transpose(delta), delta).squeeze()  # line: 13  # check
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

        # check
        # F_x_j = np.zeros((6, 2*N+3))
        # F_x_j[:3, :3] = np.eye(3)
        # # F_x_j[3, 2*j] = 1
        # F_x_j[4, 2*j+1] = 1
        # F_x_j[5, 2*j+2] = 1  # line: 15
        # # F_x_j = np.vstack((F_x_j, [0, 0, 0, 0, 0, 0, 1]))  # line: 15

        # test
        # F_x_j = np.zeros((5, 2*N+3))
        # F_x_j[:3, :3] = np.eye(3)
        # F_x_j[3, 2*j+1] = 1
        # F_x_j[4, 2*j+2] = 1

        # F_x_j[5, 2*j+2] = 1  # line: 15
        # F_x_j = np.vstack((F_x_j, [0, 0, 0, 0, 0, 0, 1]))  # line: 15
        # test/

        # 0508
        F_x_j = np.zeros((5, 2*N+3))
        F_x_j[:3, :3] = np.eye(3)
        F_x_j[3, 2*j+1] = 1
        F_x_j[4, 2*j+2] = 1  # line: 15
        # 0508/
        # check/

        # test
        print('\nF_x_j >>')
        print(F_x_j)
        # test/

        # check
        # h_t = np.array([[root_q*delta_x, -root_q*delta_y, 0, -root_q*delta_x, root_q*delta_y, 0],
        #                 [delta_y, delta_x, -1, -delta_y, -delta_x, 0],
        #                 [0, 0, 0, 0, 0, 1]])  # h_t: Jacobian of approximated linear function
        # H_t = (1/q) * np.dot(h_t, F_x_j)  # line:16  # check

        # test
        # h_t = np.array([[root_q*delta_x, -root_q*delta_y, 0, -root_q*delta_x, root_q*delta_y],
        #                 [delta_y, delta_x, -1, -delta_y, -delta_x]])  # h_t: Jacobian of approximated linear function
        # h_t = np.array([[-root_q * delta_x, -root_q * delta_y, 0, root_q * delta_x, root_q * delta_y],
        #                 [delta_y, -delta_x, -1, -delta_y, delta_x]])  # h_t: Jacobian of approximated linear function
        # H_t = (1/q) * np.dot(h_t, F_x_j)  # line:16  # check
        # test/

        # 0508
        h_t = np.array([[-root_q * delta_x, -root_q * delta_y, 0, root_q * delta_x, root_q * delta_y],
                        [delta_y, -delta_x, -q, -delta_y, delta_x]])  # h_t: Jacobian of approximated linear function
        H_t = (1 / q) * np.dot(h_t, F_x_j)  # line:16  # check
        # 0508/
        # check/

        # test
        print('\nH_t >>')
        print(H_t)
        # test/

        temp_matrix_1 = np.dot(sigma_t_bar, np.transpose(H_t))
        temp_matrix_2 = np.dot(np.dot(H_t, sigma_t_bar), np.transpose(H_t)) + Q_t

        # check
        try:
            temp_matrix_2_inverse = np.linalg.inv(temp_matrix_2)
        except np.linalg.linalg.LinAlgError:
            print('error')
            temp_matrix_2_inverse = 1/temp_matrix_2  # need to add min number???
        # check/

        K_t = np.dot(temp_matrix_1, temp_matrix_2_inverse)  # line: 17

        # test
        print('\nK_t >>')
        print(K_t)
        # test/

        # test
        # print('aaaaaaaaaaaaaaa')
        # print(np.transpose([z_t[:, j-1]]))
        # print(z_t_hat)
        z_dif = np.subtract(np.transpose([z_t[:, j-1]]), z_t_hat, order=1)
        # print('z_dif')
        # print(z_dif)
        mu_dif = mu_dif + np.dot(K_t, z_dif[:2, 0])
        # print('mu_dif')
        # print(mu_dif)
        sig_dif = sig_dif + np.dot(K_t, H_t)
        # print('sig_dif')
        # print(sig_dif)

        # z_dif = z_dif + np.subtract(z_t[:, j-1], z_t_hat)
        # print(z_dif)
        # test/

    # mu_t = mu_t_bar + np.sum(np.dot(K_t, (z_t-z_t_hat)))  # line:19
    # print('aaaaaaaaaaaaaaa')
    # print(z_t)
    # print(z_t_hat)
    # print(z_t-z_t_hat)
    # mu_t = mu_t_bar + np.dot(K_t, np.array([z_t[:2, 0]-z_t_hat[:2, 0]]).T)  # line:19
    mu_t = mu_t_bar + np.array([mu_dif]).transpose()  # line:19
    # sigma_t = np.dot((np.eye(2*N + 3) - np.sum(np.dot(K_t, H_t))), sigma_t_bar)  # line: 20
    sigma_t = np.dot((np.eye(2 * N + 3) - sig_dif), sigma_t_bar)  # line: 20

    return mu_t, sigma_t


def EKF_SLAM(mu_t_1, sigma_t_1, u_t, z_t, c_t, dt, N):
    mu_t_bar, sigma_t_bar = motion_update(mu_t_1, sigma_t_1, u_t, dt, N)
    mu_t, sigma_t = measurement_update(mu_t_bar, sigma_t_bar, z_t, N)

    return mu_t, sigma_t  # line: 21
