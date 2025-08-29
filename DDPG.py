import torch
import torch.nn as nn
import torch.optim as optim
import numpy as np
import random
from collections import deque

class Actor(nn.Module):
    def __init__(self, state_dim, action_dim, max_action):
        super(Actor, self).__init__()
        self.layer1 = nn.Linear(state_dim, 400)
        self.layer2 = nn.Linear(400, 300)
        self.layer3 = nn.Linear(300, action_dim)
        self.max_action = max_action

    def forward(self, state):
        x = torch.relu(self.layer1(state))
        x = torch.relu(self.layer2(x))
        action = torch.sigmoid(self.layer3(x))  # 输出在[0,1]
        return action * self.max_action  # 映射到[0, 6]

class Critic(nn.Module):
    def __init__(self, state_dim, action_dim):
        super(Critic, self).__init__()
        self.layer1 = nn.Linear(state_dim + action_dim, 400)
        self.layer2 = nn.Linear(400, 300)
        self.layer3 = nn.Linear(300, 1)

    def forward(self, state, action):
        # 确保动作是二维张量
        if action.dim() == 1:
            action = action.unsqueeze(1)

        x = torch.cat([state, action], 1)
        x = torch.relu(self.layer1(x))
        x = torch.relu(self.layer2(x))
        q_value = self.layer3(x)
        return q_value
    
class OUNoise:
    def __init__(self, action_dim, mu=0.0, theta=0.15, sigma=0.2):
        self.action_dim = action_dim
        self.mu = mu          # 均值
        self.theta = theta    # 回归速度系数
        self.sigma = sigma    # 随机波动强度
        self.state = np.ones(self.action_dim) * self.mu  # 初始状态
        self.reset()

    def reset(self):
        self.state = np.ones(self.action_dim) * self.mu

    def sample(self):
        dx = self.theta * (self.mu - self.state) 
        dx += self.sigma * np.random.randn(self.action_dim)
        self.state += dx
        return self.state

class DDPGAgent:
    def __init__(self, state_dim=5, action_dim=1, batch_size=64, max_action=6):
        self.state_dim = state_dim
        self.action_dim = action_dim
        self.batch_size = batch_size
        self.max_action = max_action
        self.gamma = 0.99
        self.tau = 0.0005
        self.ou_noise = OUNoise(action_dim)  # 初始化OU噪声
        self.epsilon = 1.0
        self.epsilon_min = 0.01
        self.epsilon_decay = 0.995
        
        # 初始化网络
        self.actor = Actor(state_dim, action_dim, max_action)
        self.actor_target = Actor(state_dim, action_dim, max_action)
        self.critic = Critic(state_dim, action_dim)
        self.critic_target = Critic(state_dim, action_dim)
        
        self.actor_target.load_state_dict(self.actor.state_dict())
        self.critic_target.load_state_dict(self.critic.state_dict())
        
        self.actor_optimizer = optim.Adam(self.actor.parameters(), lr=1e-4)
        self.critic_optimizer = optim.Adam(self.critic.parameters(), lr=1e-3)
        
        self.memory = deque(maxlen=10000)
        
        self.step_counter = 0
        self.step_rewards = []
        self.total_reward = 0
        self.train_step_counter = 0

    def act(self, state, explore=True):
        # state = torch.FloatTensor(state.reshape(1, -1))
        state = torch.FloatTensor(state).unsqueeze(0)
        action = self.actor(state).detach().numpy()[0][0]
        # if explore and np.random.rand() < self.epsilon:
        #     action += np.random.normal(0, 0.1)  # 高斯噪声
        # return np.clip(action, 0, self.max_action) / self.max_action  # 归一化到[0,1]
        if explore:
            noise = self.ou_noise.sample()[0]
            action += self.epsilon * noise
            action = np.clip(action, 0, self.max_action)  # 限制到[0, 6]
        return action

    def remember(self, state, action, reward, next_state, done):
        self.memory.append((state, action, reward, next_state, done))

    def replay(self):
        if len(self.memory) < self.batch_size:
            return False
        # batch = np.array(random.sample(self.memory, self.batch_size))
        # state = torch.FloatTensor(np.vstack(batch[:, 0]))
        # action = torch.FloatTensor(np.vstack(batch[:, 1]))
        # reward = torch.FloatTensor(np.vstack(batch[:, 2]))
        # next_state = torch.FloatTensor(np.vstack(batch[:, 3]))
        # done = torch.FloatTensor(np.vstack(batch[:, 4]).astype(np.float32))
        batch = random.sample(self.memory, self.batch_size)
        state, action, reward, next_state, done = zip(*batch)

        state = torch.FloatTensor(state)
        action = torch.FloatTensor(action)
        reward = torch.FloatTensor(reward)
        next_state = torch.FloatTensor(next_state)
        done = torch.FloatTensor(done)
        
        # 计算目标Q值
        next_action = self.actor_target(next_state)
        q_next = self.critic_target(next_state, next_action.detach())
        target_q = reward + (1 - done) * self.gamma * q_next
        
        # 更新Critic
        current_q = self.critic(state, action)
        critic_loss = nn.MSELoss()(current_q, target_q.detach())
        self.critic_optimizer.zero_grad()
        critic_loss.backward()
        self.critic_optimizer.step()
        
        # 更新Actor
        actor_loss = -self.critic(state, self.actor(state)).mean()
        self.actor_optimizer.zero_grad()
        actor_loss.backward()
        self.actor_optimizer.step()
        
        # 软更新目标网络
        for target_param, param in zip(self.actor_target.parameters(), self.actor.parameters()):
            target_param.data.copy_(self.tau * param.data + (1 - self.tau) * target_param.data)
        for target_param, param in zip(self.critic_target.parameters(), self.critic.parameters()):
            target_param.data.copy_(self.tau * param.data + (1 - self.tau) * target_param.data)
        
        # 衰减探索率
        if self.epsilon > self.epsilon_min:
            self.epsilon *= self.epsilon_decay
        return True
    
    def save_model(self, actor_path="ddpg_actor.pth", critic_path="ddpg_critic.pth"):
        torch.save(self.actor.state_dict(), actor_path)
        torch.save(self.critic.state_dict(),critic_path)

    def load_model(self, actor_path="ddpg_actor.pth", critic_path="ddpg_critic.pth"):
        # 实例化
        self.actor = Actor(self.state_dim, self.action_dim, self.max_action)
        self.actor_target = Actor(self.state_dim, self.action_dim, self.max_action)
        self.critic = Critic(self.state_dim, self.action_dim)
        self.critic_target = Critic(self.state_dim, self.action_dim)
        # 加载
        self.actor.load_state_dict(torch.load(actor_path))
        self.critic.load_state_dict(torch.load(critic_path))
        self.actor_target.load_state_dict(self.actor.state_dict())
        self.critic_target.load_state_dict(self.critic.state_dict())

        # 设置评估模式
        [net.eval() for net in [self.actor, self.actor_target, 
                            self.critic, self.critic_target]]

# 全局代理对象
_agent = None

def init_agent(state_dim, action_dim):
    global _agent
    _agent = DDPGAgent(state_dim, action_dim)
    return True

def get_action(state,explore=True):
    if _agent is None:
        return 0.0
    return _agent.act(state,explore)

def learn(state, action, reward, next_state, done):
    if _agent is None:
        return False
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
        'step_rewards': _agent.step_rewards
    }

def save_model(actor_path="ddpg_actor.pth", critic_path="ddpg_critic.pth"):
    """ 调用 agent 的保存方法 """
    if _agent is None:
        raise ValueError("Agent 未初始化")
    _agent.save_model(actor_path, critic_path)
    return True

def load_model(actor_path="ddpg_actor.pth", critic_path="ddpg_critic.pth"):
    if _agent is None:
        raise ValueError("Agent 未初始化")
    _agent.load_model(actor_path, critic_path)
    return True

def reset_episode():
    if _agent is not None:
        _agent.total_reward = 0
        _agent.step_rewards = []
    return True