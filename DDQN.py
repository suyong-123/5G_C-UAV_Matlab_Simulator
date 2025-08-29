import torch
import torch.nn as nn
import torch.optim as optim
import numpy as np
import random
from collections import deque

# === 网络结构定义 ===
class DQN(nn.Module):
    def __init__(self, input_dim, output_dim):
        super(DQN, self).__init__()
        self.fc1 = nn.Linear(input_dim, 128)
        self.norm1 = nn.LayerNorm(128)
        self.fc2 = nn.Linear(128, 128)
        self.norm2 = nn.LayerNorm(128)
        self.fc3 = nn.Linear(128, output_dim)

    def forward(self, x):
        x = torch.relu(self.norm1(self.fc1(x)))
        x = torch.relu(self.norm2(self.fc2(x)))
        return self.fc3(x)

class DuelingDQN(nn.Module):
    def __init__(self, input_dim, output_dim):
        super(DuelingDQN, self).__init__()
        self.feature = nn.Sequential(
            nn.Linear(input_dim, 128),
            nn.ReLU()
        )
        self.value_stream = nn.Sequential(
            nn.Linear(128, 128),
            nn.ReLU(),
            nn.Linear(128, 1)
        )
        self.advantage_stream = nn.Sequential(
            nn.Linear(128, 128),
            nn.ReLU(),
            nn.Linear(128, output_dim)
        )

    def forward(self, x):
        x = self.feature(x)
        value = self.value_stream(x)
        advantage = self.advantage_stream(x)
        return value + advantage - advantage.mean(dim=1, keepdim=True)

# === 智能体类 ===
class DoubleDQNAgent:
    def __init__(self, state_dim, action_dim, learning_rate, gamma, epsilon, 
                 epsilon_decay, epsilon_min, batch_size, memory_size, target_update_freq,
                 use_dueling=False, n_step=1):
        self.state_dim = state_dim
        self.action_dim = action_dim
        self.learning_rate = learning_rate
        self.gamma = gamma
        self.epsilon = epsilon
        self.epsilon_decay = epsilon_decay
        self.epsilon_min = epsilon_min
        self.batch_size = batch_size
        self.memory = deque(maxlen=memory_size)
        self.target_update_freq = target_update_freq
        self.total_reward = 0
        self.train_step_counter = 0
        self.step_rewards = []

        self.use_dueling = use_dueling
        self.n_step = n_step
        self.n_step_buffer = deque(maxlen=n_step)

        NetClass = DuelingDQN if use_dueling else DQN
        self.online_net = NetClass(state_dim, action_dim)
        self.target_net = NetClass(state_dim, action_dim)
        self.target_net.load_state_dict(self.online_net.state_dict())
        self.target_net.eval()

        self.optimizer = optim.Adam(self.online_net.parameters(), lr=learning_rate)
        self.criterion = nn.MSELoss()

    def remember(self, state, action, reward, next_state, done):
        self.n_step_buffer.append((state, action, reward, next_state, done))
        if len(self.n_step_buffer) < self.n_step:
            return
        R, S, A = 0, self.n_step_buffer[0][0], self.n_step_buffer[0][1]
        for idx, (_, _, r, _, d) in enumerate(self.n_step_buffer):
            R += (self.gamma ** idx) * r
            if d:
                break
        next_s, done_flag = self.n_step_buffer[-1][3], self.n_step_buffer[-1][4]
        self.memory.append((S, A, R, next_s, done_flag))

    def act(self, state):
        if np.random.rand() <= self.epsilon:
            return random.randrange(self.action_dim)
        state = torch.FloatTensor(state).unsqueeze(0)
        with torch.no_grad():
            q_values = self.online_net(state)
        return torch.argmax(q_values).item()

    def replay(self):
        if len(self.memory) < self.batch_size:
            return
        minibatch = random.sample(self.memory, self.batch_size)
        states, actions, rewards, next_states, dones = zip(*minibatch)

        states = torch.FloatTensor(states)
        actions = torch.LongTensor(actions)
        rewards = torch.FloatTensor(rewards)
        next_states = torch.FloatTensor(next_states)
        dones = torch.FloatTensor(dones)

        current_q = self.online_net(states).gather(1, actions.unsqueeze(1)).squeeze(1)
        with torch.no_grad():
            online_next_actions = self.online_net(next_states).argmax(dim=1)
            target_next_q = self.target_net(next_states).gather(1, online_next_actions.unsqueeze(1)).squeeze(1)
            target_q = rewards + (1 - dones) * (self.gamma ** self.n_step) * target_next_q

        loss = self.criterion(current_q, target_q)
        self.optimizer.zero_grad()
        loss.backward()
        self.optimizer.step()

        self.train_step_counter += 1
        if self.train_step_counter % self.target_update_freq == 0:
            self.target_net.load_state_dict(self.online_net.state_dict())

        if self.epsilon > self.epsilon_min:
            self.epsilon *= self.epsilon_decay

    def save_checkpoint(self, path="dqn_checkpoint.pth"):
        checkpoint = {
            'online_net_state_dict': self.online_net.state_dict(),
            'target_net_state_dict': self.target_net.state_dict(),
            'optimizer_state_dict': self.optimizer.state_dict(),
            'memory': list(self.memory),
            'epsilon': self.epsilon,
            'train_step_counter': self.train_step_counter,
            'total_reward': self.total_reward,
            'step_rewards': self.step_rewards,
            'state_dim': self.state_dim,
            'action_dim': self.action_dim
        }
        torch.save(checkpoint, path)
        return True

    def load_checkpoint(self, path="dqn_checkpoint.pth"):
        checkpoint = torch.load(path)
        if self.state_dim != checkpoint['state_dim'] or self.action_dim != checkpoint['action_dim']:
            raise ValueError("状态或动作维度不匹配！")
        self.online_net.load_state_dict(checkpoint['online_net_state_dict'])
        self.target_net.load_state_dict(checkpoint['target_net_state_dict'])
        self.optimizer.load_state_dict(checkpoint['optimizer_state_dict'])
        self.memory = deque(checkpoint['memory'], maxlen=self.memory.maxlen)
        self.epsilon = checkpoint['epsilon']
        self.train_step_counter = checkpoint['train_step_counter']
        self.total_reward = checkpoint['total_reward']
        self.step_rewards = checkpoint['step_rewards']
        return True

    def save_model(self, path="dqn_model.pth"):
        torch.save(self.online_net.state_dict(), path)
        return True

    def load_model(self, path="dqn_model.pth"):
        self.online_net.load_state_dict(torch.load(path))
        self.target_net.load_state_dict(self.online_net.state_dict())
        self.epsilon = self.epsilon_min
        return True

# === 接口封装 ===
_agent = None

def init_agent(state_dim, action_dim, config):
    global _agent
    _agent = DoubleDQNAgent(
        state_dim=state_dim,
        action_dim=action_dim,
        learning_rate=config['learning_rate'],
        gamma=config['gamma'],
        epsilon=config['epsilon'],
        epsilon_decay=config['epsilon_decay'],
        epsilon_min=config['epsilon_min'],
        batch_size=config['batch_size'],
        memory_size=config['memory_size'],
        target_update_freq=config['target_update_freq'],
        use_dueling=config.get('use_dueling', False),
        n_step=config.get('n_step', 1)
    )
    return True

def get_action(state):
    if _agent is None:
        raise ValueError("Agent 未初始化")
    return _agent.act(state)

def learn(state, action, reward, next_state, done):
    if _agent is None:
        raise ValueError("Agent 未初始化")
    _agent.remember(state, action, reward, next_state, done)
    _agent.replay()
    _agent.total_reward += reward
    _agent.step_rewards.append(reward)
    return True

def get_agent_info():
    if _agent is None:
        return {}
    return {
        'state_dim': _agent.state_dim,
        'action_dim': _agent.action_dim,
        'epsilon': _agent.epsilon,
        'train_steps': _agent.train_step_counter,
        'total_reward': _agent.total_reward,
        'step_rewards': _agent.step_rewards,
        'memory_size': len(_agent.memory)
    }

def save_checkpoint(path="dqn_checkpoint.pth"):
    if _agent is None:
        raise ValueError("Agent 未初始化")
    return _agent.save_checkpoint(path)

def load_checkpoint(path="dqn_checkpoint.pth"):
    if _agent is None:
        raise ValueError("Agent 未初始化，请先调用 init_agent()")
    return _agent.load_checkpoint(path)

def save_model(path="dqn_model.pth"):
    if _agent is None:
        raise ValueError("Agent 未初始化")
    return _agent.save_model(path)

def load_model(path="dqn_model.pth"):
    if _agent is None:
        raise ValueError("Agent 未初始化，请先调用 init_agent()")
    return _agent.load_model(path)

def reset_episode():
    if _agent is not None:
        _agent.total_reward = 0
        _agent.step_rewards = []
    return True
