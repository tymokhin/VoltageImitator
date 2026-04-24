%% Параметри сигналу
fs    = 50e3;        % частота дискретизації, Гц
f0    = 50;          % частота синусоїди, Гц
omega = 2*pi*f0;
phi   = 20*pi/180;   % початкова фаза, рад

A0    = 1.0;         % початкова амплітуда
alpha = 5;           % коеф. експоненти (1/с), A(t) = A0*exp(alpha*t)

Tsim  = 0.4;         % тривалість моделювання, с (наприклад 0.4с ~ 20 періодів)
t     = (0:1/fs:Tsim-1/fs).';   % час, стовпчиковий вектор

%% Формуємо експоненційну огинаючу та сигнал
A_true = A0 * exp(alpha * t);              % A(t) = A0*exp(alpha*t)
x      = A_true .* sin(omega*t + phi);     % x(t) = A(t)*sin(wt+phi)

% (опціонально можна додати шум, щоб побачити стійкість методу)
addNoise = true;
if addNoise
    x = x + 0.1*randn(size(x));           % білий шум
end

%% Вікно синхронного детектора
% Візьмемо 1 період як вікно. Можеш замінити на fs/f0/2 для півперіоду.
Ns_per = round(fs / (2*f0));    % кількість відліків на один період 50 Гц
T_per  = Ns_per / fs;

% Опорні sin/cos (база) на один період
n_win  = (0:Ns_per-1).';
sin_ref = sin(omega * n_win / fs);
cos_ref = cos(omega * n_win / fs);

% Нормувальний коефіцієнт:
% Для чистого x = A*sin(wt+phi):
% S_sin = (N/2)*A*cos(phi), S_cos = (N/2)*A*sin(phi)
% => sqrt(S_sin^2 + S_cos^2) = (N/2)*A
% => A = 2/N * sqrt(S_sin^2 + S_cos^2)
normK = 2 / Ns_per;

%% Оцінка амплітуди "на льоту" блоками по одному періоду
nBlocks = floor(length(x) / Ns_per);

A_est   = zeros(nBlocks, 1);
t_est   = zeros(nBlocks, 1);

for k = 1:nBlocks
    idx_start = (k-1)*Ns_per + 1;
    idx_end   = k*Ns_per;
    
    x_block = x(idx_start:idx_end);
    
    % Синхронне множення та сумування
    S_sin = sum(x_block .* sin_ref);
    S_cos = sum(x_block .* cos_ref);
    
    % Оцінена амплітуда для цього блоку
    A_est(k) = normK * sqrt(S_sin^2 + S_cos^2);
    
    % Часовий момент (центр вікна)
    t_est(k) = (idx_start + idx_end - 1)/(2*fs);
end

% Істинна амплітуда в ті самі моменти
A_true_at_est = A0 * exp(alpha * t_est);

%% Графіки
figure;
subplot(2,1,1);
plot(t, x);
grid on;
xlabel('t, s');
ylabel('x(t)');
title('Сигнал x(t) = A_0 e^{\alpha t} \cdot sin(\omega t + \phi)');

subplot(2,1,2);
plot(t_est, A_true_at_est, 'LineWidth', 1.5); hold on;
plot(t_est, A_est, '--o', 'MarkerSize', 4);
grid on;
xlabel('t, s');
ylabel('Amplitude');
legend('A_{true}(t)', 'A_{est}(t)', 'Location', 'northwest');
title('Істинна vs оцінена амплітуда (синхронний детектор)');

%% Вивід кількох чисел у консоль
disp('   t_est      A_true       A_est');
disp([t_est(1:10) A_true_at_est(1:10) A_est(1:10)]);