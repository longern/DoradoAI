#include "sdk.h"
#include "const.h"
#include "console.h"
#include "filter.h"
#include <cstdio>
#include <cstdlib>
#include <vector>
#include <set>
#include <memory>
#include <queue>
#include <cmath>
using namespace std;

#define myCon AIController::ins()->console()

class Strategy;
class AIHero;

class AIController //����ai������
{
public:
	AIController(const PMap &map, const PPlayerInfo &info, PCommand &cmd);
	~AIController() { delete _console; }
public:
	void globalStrategy();
	void levelupHero();
	void buyNewHero();
	void assignBaseAttack();
	void action();

	static AIController *ins() { return instance; }

	Console *console() { return _console; }

	const std::vector<shared_ptr<AIHero>> &decisionPool() { return AIHeroList; }
	const std::vector<Strategy *> &strategyPool() { return allStrategy; }

private:
	static AIController *instance;

	Console *_console;
	const PMap *map;
	const PPlayerInfo *info;
	PCommand *cmd;

	vector<shared_ptr<AIHero>> AIHeroList;
	vector<Strategy *> allStrategy;

public:
	vector<PUnit*> myHeros, enemyHeros, monsters;
	PUnit *myBase, *enemyBase;
};

AIController *AIController::instance = nullptr;
/********************************************************************************/


/********************************* Constants ************************************/
namespace lsy
{
	static int heroRank = 1234;
	static int newHeroFirst = 1;
	static int attackArg = 200, supportRange = 600, monsterAvoid = 50, enemyAvoid = 30, deltaFightArg = 100;
	static int outOfRangeArg = 100;
	static int miningArg = 100, sameMineArg = 10, centerMineArg = 30;
	static int goBackHomeHp = 100, goBackHomeArg = 300;
	static int hammerDizzy = 50;
	static int strategyDisabled = -1 << 30;
	static bool bePushedMyBase = true;

	static const std::vector<std::string> HERO_NAME = { "Hammerguard", "Master", "Berserker", "Scouter" };
	static const std::set<std::string> ACTIVE_SKILLS = { "HammerAttack", "Blink", "Sacrifice", "SetObserver" };

	static const std::string &strHammerguard = HERO_NAME[0];
	static const std::string strMaster = HERO_NAME[1];
	static const std::string strBerserker = HERO_NAME[2];
	static const std::string strScouter = HERO_NAME[3];
}

using namespace lsy;
/********************************************************************************/


/******************************* Simple Functions *******************************/
inline bool isEqual(const char *str1, const char *str2) { return !strcmp(str1, str2); }

bool isHero(string unitName)
{
	return find(HERO_NAME.begin(), HERO_NAME.end(), unitName) != HERO_NAME.end();
}

bool isMonster(string unitName)
{
	return unitName == "Dragon" || unitName == "Roshan";
}

bool isActiveSkill(PSkill* mySkill) //�жϼ����Ƿ�Ϊ��������
{
	return ACTIVE_SKILLS.count(mySkill->name) != 0;
}

Pos campRotate(int x, int y)
{
	if (myCon->camp())
		return Pos(149 - x, 149 - y);
	else
		return Pos(x, y);
}

Pos campRotate(Pos p)
{
	if (myCon->camp())
		return Pos(149 - p.x, 149 - p.y);
	else
		return p;
}

int friendsInRange(PUnit *worker)
{
	int res = 0;
	UnitFilter filter;
	filter.setAreaFilter(new Circle(worker->pos, worker->view));
	for (auto x : myCon->friendlyUnits(filter))
		if(x->isHero() || x->isBase())
			++res;
	return res;
}

int enemiesInRange(PUnit *worker)
{
	int res = 0;
	UnitFilter filter;
	filter.setAreaFilter(new Circle(worker->pos, worker->view));
	for (auto x : myCon->enemyUnits(filter))
		if (x->isHero())
			++res;
	return res;
}
/********************************************************************************/


/******************************** Path Function *********************************/
void findPath(const PMap &map, Pos start, Pos dest, const vector<Pos> &blocks, vector<Pos> &path)
{
	struct Node {
		int x, y;
		int level;
		int priority;

		Node() :x(0), y(0), level(0), priority(0) {}
		Node(int xp, int yp, int d, int p) :x(xp), y(yp), level(d), priority(p) {}
		void updatePriority(int xDest, int yDest)
		{
			priority = level + estimate(xDest, yDest) * 10;
		}
		void nextLevel()
		{
			level += 10;
		}
		int estimate(const int & xDest, const int & yDest) const
		{
			int xd = xDest - x, yd = yDest - y;
			return static_cast<int>(sqrt(xd * xd + yd * yd));
		}
		bool operator < (const Node &b) const
		{
			return priority > b.priority;
		}
	};
	const int dx[] = { 1, 0, -1, 0 };
	const int dy[] = { 0, 1, 0, -1 };
	static int closed_nodes_map[MAP_SIZE][MAP_SIZE];
	static int open_nodes_map[MAP_SIZE][MAP_SIZE];
	static int dir_map[MAP_SIZE][MAP_SIZE];
	priority_queue<Node> pq[2];
	int pqi;
	int i, j, x, y, xdx, ydy;
	memset(closed_nodes_map, 0, sizeof(closed_nodes_map));
	memset(open_nodes_map, 0, sizeof(open_nodes_map));
	if (start == dest)
		return;
	for (size_t i = 0; i < blocks.size(); ++i)
		if (blocks[i].x >= 0 && blocks[i].x < MAP_SIZE && blocks[i].y >= 0 && blocks[i].y < MAP_SIZE)
			closed_nodes_map[blocks[i].x][blocks[i].y] = 1;
	if (closed_nodes_map[dest.x][dest.y] == 1) {
		xdx = start.x < dest.x ? -1 : 1;
		ydy = start.y < dest.y ? -1 : 1;
		x = dest.x;
		y = dest.y;
		while (closed_nodes_map[x][y] == 1) {
			if (checkOnLine(start, dest, Pos(x + xdx, y)))
				x += xdx;
			else
				y += ydy;
		}
		dest = Pos(x, y);
	}
	Node n0(start.x, start.y, 0, 0);
	n0.updatePriority(dest.x, dest.y);
	pqi = 0;
	pq[pqi].push(n0);
	open_nodes_map[start.x][start.y] = n0.priority;
	while (!pq[pqi].empty()) {
		n0 = pq[pqi].top();
		pq[pqi].pop();
		x = n0.x;
		y = n0.y;
		open_nodes_map[x][y] = 0;
		closed_nodes_map[x][y] = 1;
		if (x == dest.x && y == dest.y) {
			while (!(x == start.x && y == start.y)) {
				j = dir_map[x][y];
				path.push_back(Pos(x, y));
				x += dx[j];
				y += dy[j];
			}
			path.push_back(start);
			reverse(path.begin(), path.end());
			return;
		}
		for (i = 0; i < 4; i++) {
			xdx = x + dx[i];
			ydy = y + dy[i];
			if (!(xdx < 0 || xdx >= MAP_SIZE || ydy < 0 || ydy >= MAP_SIZE
				|| map.getHeight(xdx, ydy) - map.getHeight(x, y) > 1
				|| map.getHeight(xdx, ydy) - map.getHeight(x, y) < -1
				|| closed_nodes_map[xdx][ydy] == 1)) {
				Node m0(n0);
				m0.x = xdx;
				m0.y = ydy;
				m0.nextLevel();
				m0.updatePriority(dest.x, dest.y);
				if (open_nodes_map[xdx][ydy] == 0) {
					open_nodes_map[xdx][ydy] = m0.priority;
					pq[pqi].push(m0);
					dir_map[xdx][ydy] = (i + 2) % 4;
				}
				else if (open_nodes_map[xdx][ydy] > m0.priority) {
					open_nodes_map[xdx][ydy] = m0.priority;
					dir_map[xdx][ydy] = (i + 2) % 4;
					while (!(pq[pqi].top().x == xdx && pq[pqi].top().y == ydy)) {
						pq[1 - pqi].push(pq[pqi].top());
						pq[pqi].pop();
					}
					pq[pqi].pop();
					if (pq[pqi].size() > pq[1 - pqi].size())
						pqi = 1 - pqi;
					while (!pq[pqi].empty()) {
						pq[1 - pqi].push(pq[pqi].top());
						pq[pqi].pop();
					}
					pqi = 1 - pqi;
					pq[pqi].push(m0);
				}
			}
		}
	}
}
/********************************** Strategies **********************************/


/********************************** Strategies **********************************/
class Strategy //���Ի���
{
public:
	Strategy() { }
	Strategy(PUnit *worker, std::string name) : worker(worker), name(name) { }
	virtual int countWorth() = 0; //����ĳ�ֲ��Ե�ʵ�м�ֵ
	virtual void work() = 0; //ʵ�����ֲ���

public:
	void setWorker(PUnit* worker) { this->worker = worker; }
	int getWorth() { return worth; }
	void setWorth(int worth) { this->worth = worth; }
	PUnit* getWorker() { return worker; }
	std::string getName() { return name; }

protected:
	PUnit *worker;
	int worth;
	std::string name;
};

class AIHero //��Ӧÿ��Ӣ�۵Ĳ��Ծ���
{
public:
	AIHero(PUnit* newHero) { mHero = newHero; mStrategy = NULL; } //Ĭ��ѡ��ؼҵĲ���
	~AIHero() { if (mStrategy != NULL) delete mStrategy; }

public:
	void chooseStrategy(const vector<Strategy *> strats)
	{
		int maxWorth = -1000000000;
		for (auto x : strats)
		{
			if (x->getWorker() != this->mHero)
				continue;
			int stratWorth = x->countWorth();
			if (stratWorth > maxWorth)
				mStrategy = x, maxWorth = stratWorth;
		}
	}

	void action() { if(mStrategy) mStrategy->work(); } //���е�ǰ�����Ų���
	PUnit *getHero() { return mHero; }
	Strategy *getStrategy() { return mStrategy; }
private:
	PUnit *mHero;
	Strategy *mStrategy;
};

class Unavalable : public Strategy
{
public:
	Unavalable(PUnit* worker) : Strategy(worker, "Unavalable") { }
public:
	int countWorth()
	{
		if (worker->findBuff("Dizzy") != nullptr)
		{
			name = "Dizzy";
			return worth = 2147483640;
		}
		if (worker->findBuff("Reviving") != nullptr)
		{
			name = "Reviving";
			return worth = 2147483640;
		}
		return worth = strategyDisabled;
	}
	void work() { }
};

class Attack : public Strategy //����ĳ����λ�Ĳ���
{
public:
	Attack(PUnit* worker, PUnit* target) : Strategy(worker, "Attack") { setTarget(target); }
public:
	int countWorth()
	{
		this->worth = 0;
		if (target->findBuff("Reviving") != NULL || target->hp <= 0)
			this->worth = strategyDisabled;

		if (dis2(worker->pos, target->pos) <= supportRange)
		{
			this->worth += int(((double)target->max_hp / target->hp) * attackArg);
			if (dis2(target->pos, AIController::ins()->myBase->pos) <= AIController::ins()->myBase->view)
				worth += 300;
			if (worker->findBuff("WinOrDie"))
				worth += 2000;
			if (dis2(worker->pos, target->pos) <= worker->range)
			{
				if (target->name == strBerserker)
				{
					if (target->canUseSkill("Sacrifice"))
						worth += 300;
					else
						worth -= 20;
				}
				if (worker->atk - target->def >= target->hp)
					worth += 3000;
			}
			else
			{
				this->worth -= outOfRangeArg;
			}
			if (target->isWild()) this->worth -= monsterAvoid;
		}
		return this->worth;
	}

	void work() {
		myCon->selectUnit(worker);
		myCon->attack(target);
	}

	void setTarget(PUnit* target) { this->target = target; }
private:
	PUnit* target;
};

class PlugEye : public Strategy  // ���۲���
{
public:
	PlugEye(PUnit* worker) : Strategy(worker, "PlugEye") { }
public:
	void sklSetObserver(PSkill *skill)
	{
		this->worth = 0;
		if (myCon->round() < 30)
			return;
		int fHeroNearCenter = 0;
		UnitFilter filter;
		filter.setAreaFilter(new Circle(MINE_POS[0], 225));
		for (auto x : myCon->enemyUnits(filter))
		{
			if (x->isHero())
				return;
		}
		for (auto x : AIController::ins()->myHeros)
		{
			if (x != worker && dis2(x->pos, MINE_POS[0]) < 144 && x->findBuff("BeAttacked") == nullptr)
				fHeroNearCenter++;
		}
		if (fHeroNearCenter >= 2)
		{
			targetPos = campRotate(99, 84);
			this->worth += 100;
		}
	}

	int countWorth()
	{
		this->worth = 0;
		myCon->selectUnit(worker);
		if (!worker->canUseSkill("SetObserver"))
			return this->worth = strategyDisabled;
		else
			sklSetObserver(myCon->getSkill("SetObserver", worker));
		return this->worth;
	}

	void work()
	{
		myCon->selectUnit(worker);
		if (dis2(worker->pos, targetPos) <= 25)
			myCon->useSkill(myCon->getSkill("SetObserver", worker), targetPos);
		else
			myCon->move(targetPos);
	}

private:
	Pos targetPos;
};

class UseSkill : public Strategy  // ��ĳ����λ�ͷż��ܵĲ���
{
public:
	UseSkill(PUnit* worker, PUnit* target) : Strategy(worker, "UseSkill") { setTarget(target); }
public:
	void sklHammerAttack()
	{
		if (dis2(worker->pos, target->pos) <= supportRange)
		{
			if (target->hp <= 0)
				return;
			this->worth += int(((double)target->max_hp / target->hp) * attackArg);
			if (dis2(worker->pos, target->pos) > HAMMERATTACK_RANGE)
				this->worth -= outOfRangeArg;
			if (target->findBuff("Dizzy") != NULL)
				this->worth -= hammerDizzy;
		}
	}

	void sklSacrifice()
	{
		if (target->isWild())
			this->worth -= 2000;
		if (dis(worker->pos, target->pos) <= sqrt(worker->range) + sqrt(worker->speed) / 2)
		{
			this->worth += int(((double)target->max_hp / target->hp) * attackArg * 2);
			if (dis2(worker->pos, target->pos) > worker->view)
				this->worth -= outOfRangeArg * 2;
			if (target->findBuff("Dizzy") != NULL)
				this->worth += hammerDizzy;
		}
	}

	int countWorth()
	{
		this->worth = 0;
		if (target->hp <= 0)
			return this->worth;
		myCon->selectUnit(worker);
		vector<PSkill*> skillList = myCon->getSkills();
		for (size_t i = 0; i < skillList.size(); ++i)
			if (isActiveSkill(skillList[i]))
			{
				if (!worker->canUseSkill(skillList[i]))
				{
					this->worth = strategyDisabled;
					break;
				}
				if (isEqual(skillList[i]->name, "HammerAttack"))
					sklHammerAttack();
				else if (isEqual(skillList[i]->name, "Sacrifice"))
					sklSacrifice();
				break;
			}
		return this->worth;
	}

	void work()
	{
		myCon->selectUnit(worker);
		vector<PSkill*> skillList = myCon->getSkills();
		for (size_t i = 0; i < skillList.size(); ++i)
			if (isActiveSkill(skillList[i]))
			{
				if (isEqual(skillList[i]->name, "HammerAttack"))
					myCon->useSkill(skillList[i], target);
				else if (isEqual(skillList[i]->name, "Sacrifice"))
					myCon->useSkill(skillList[i], NULL);
				else
					myCon->useSkill(skillList[i], targetPos);
				break;
			}
	}

	void setTarget(PUnit* target) { this->target = target; }
private:
	PUnit* target;
	Pos targetPos;
};

class GoBackHome : public Strategy //Ӣ�۲��лؼҵĲ���
{
public:
	GoBackHome(PUnit* worker) : Strategy(worker, "GoBackHome") { }
public:
	int countWorth()
	{
		this->worth = 0;
		if (worker->hp <= goBackHomeHp && strcmp(worker->name, "Berserker") != 0)
			this->worth += goBackHomeArg;
		if (friendsInRange(worker) < AIController::ins()->myHeros.size()
			&& friendsInRange(worker) + 1 < enemiesInRange(worker))
			this->worth += 400 * pow(2, enemiesInRange(worker) - friendsInRange(worker));
		return this->worth;
	}

	void work()
	{
		myCon->selectUnit((const PUnit*)worker);
		myCon->move(myCon->getMilitaryBase()->pos);
	}
};

class CallBackHome : public Strategy //�ٻ�Ӣ�۵Ĳ���
{
public:
	CallBackHome(PUnit* worker) : Strategy(worker, "CallBackHome") { setWorker(worker); name = "CallBackHome"; }
public:
	int countWorth()
	{
		this->worth = 0;
		if (dis2(worker->pos, AIController::ins()->myBase->pos) <= MILITARY_BASE_RANGE)
			return this->worth;
		int enemyAttackBase = 0;
		int friendProtectBase = 0;
		UnitFilter filter;
		filter.setAreaFilter(new Circle(AIController::ins()->myBase->pos, MILITARY_BASE_RANGE));
		for (auto x : myCon->enemyUnits(filter))
			enemyAttackBase++;
		for (auto x : myCon->friendlyUnits(filter))
			if(x->isHero())
				friendProtectBase++;
		for (auto x : AIController::ins()->decisionPool())
			if (x->getStrategy() && x->getStrategy()->getName() == "CallBackHome")
				friendProtectBase++;

		if (enemyAttackBase > 0)
			bePushedMyBase = true;

		if (enemyAttackBase > friendProtectBase)
			this->worth += 300;
		return this->worth;
	}
	void work() { myCon->callBackHero(worker); }
};

class GoCenterMining : public Strategy //�ɿ�Ĳ���
{
public:
	GoCenterMining(PUnit* worker) : Strategy(worker, "GoCenMine") { }
public:
	int countWorth()
	{
		this->worth = miningArg;
		if (myCon->round() < 18)
			return this->worth = 100000000;

		const auto &currentDecisionPool = AIController::ins()->decisionPool();
		for (size_t i = 0; i < currentDecisionPool.size(); ++i)
			if (currentDecisionPool[i]->getStrategy() != NULL)
				if (currentDecisionPool[i]->getStrategy()->getName() == "GoCenterMining")
					this->worth -= sameMineArg;

		this->worth -= int(dis(worker->pos, MINE_POS[0]) / 5);
		this->worth += centerMineArg;
		return this->worth;
	}

	void work() {
		myCon->selectUnit(worker);
		myCon->move(MINE_POS[0]);
	}
private:
};

class GoMining : public Strategy //�ɿ�Ĳ���
{
public:
	GoMining(PUnit* worker, Pos target) : Strategy(worker, "GoMining") { setTarget(target); }
public:
	int countWorth()
	{
		this->worth = 0;
		return this->worth;
	}

	void work() {
		myCon->selectUnit(worker);
		myCon->move(target);
	}
	void setTarget(Pos target) { this->target = target; }
	Pos getTarget() { return target; }
private:
	Pos target;
};

class Tempt : public Strategy  // ���յ��ˣ�Ұ�֣��Ĳ���-��������֤�Լ������˵�����½���hit-and-run
{
public:
	Tempt(PUnit* worker, PUnit* target) : Strategy(worker, "Tempt") { setTarget(target); }
public:
	int countWorth()
	{
		return this->worth = 0;
		if(worker->findSkill("Attack") && worker->findSkill("Attack")->cd > 0)
			this->worth = 300;
		return this->worth;
	}
	void work() {
		myCon->selectUnit(worker);
		myCon->attack(target);
	}
	void setTarget(PUnit* target) { this->target = target; }
private:
	PUnit* target;
};

class PushBase : public Strategy  // ���ϼҲ���
{
public:
	PushBase(PUnit *worker) : Strategy(worker, "PushBase") { }
public:
	int countWorth()
	{
		this->worth = 0;
		if (!bePushedMyBase)
			return this->worth;
		if (AIController::ins()->myHeros.size() >= HERO_LIMIT - 1)
			if (AIController::ins()->enemyBase && dis2(AIController::ins()->enemyBase->pos, worker->pos) <= 225)
				this->worth += 10000;
			else
				this->worth += 200;
		return this->worth;
	}
	void work()
	{
		myCon->selectUnit(worker);
		if (AIController::ins()->enemyBase)
			myCon->attack(AIController::ins()->enemyBase);
		else
		{
			int friendHeroCount = 0;
			for (auto x : AIController::ins()->myHeros)
				if (dis2(x->pos, worker->pos) <= worker->range || campRotate(x->pos).y >= 110)
					friendHeroCount++;

			if(campRotate(worker->pos).y <= 50 && campRotate(worker->pos).x <= 110)
				myCon->move(campRotate(116, 46));
			else if(friendHeroCount >= HERO_LIMIT - 1
				|| worker->pos.y > 120
				|| enemiesInRange(AIController::ins()->myBase) > 0)
				myCon->move(MILITARY_BASE_POS[1 - myCon->camp()]);
			else
				myCon->move(campRotate(138, 115));
		}
	}
};
/********************************************************************************/


/********************************************************************************/
AIController::AIController(const PMap &map, const PPlayerInfo &info, PCommand &cmd)
{
	instance = this;
	_console = new Console(map, info, cmd);
	//_console->changeShortestPathFunc(findPath);

	this->map = &map;
	this->info = &info;
	this->cmd = &cmd;
	myHeros.clear();
	std::vector<PUnit*> myUnits = myCon->friendlyUnits();
	std::vector<PUnit*> enemyUnits = myCon->enemyUnits();
	enemyBase = NULL;
	for (size_t i = 0; i < myUnits.size(); ++i)
	{
		if (isHero(myUnits[i]->name))
			myHeros.push_back(myUnits[i]);
		else if (isEqual(myUnits[i]->name, "MilitaryBase"))
			myBase = myUnits[i];
	}
	for (size_t i = 0; i < enemyUnits.size(); ++i)
	{
		if (isHero(enemyUnits[i]->name) || isMonster(enemyUnits[i]->name))
			enemyHeros.push_back(enemyUnits[i]);
		else if (isEqual(enemyUnits[i]->name, "MilitaryBase"))
			enemyBase = enemyUnits[i];
	}
}

void AIController::globalStrategy()
{
	if (newHeroFirst)
	{
		ins()->buyNewHero();
		ins()->levelupHero();
	}
	else
	{
		ins()->levelupHero();
		ins()->buyNewHero();
	}

	
}

void AIController::levelupHero()
{
	UnitFilter filter;
	filter.setAreaFilter(new Circle(myBase->pos, LEVELUP_RANGE), "a");
	vector<PUnit*> heroToLevelUp = myCon->friendlyUnits(filter);

	int heroCount = 0;
	for (auto x : myCon->friendlyUnits())
		if (x->isHero())
			heroCount++;

	for (size_t i = 0; i < heroToLevelUp.size(); ++i)
		if(heroCount == HERO_LIMIT
			|| heroToLevelUp[i]->level < 2 && heroToLevelUp[i]->findBuff("Reviving") == nullptr)
			myCon->buyHeroLevel(heroToLevelUp[i]);
}

void AIController::buyNewHero()
{
	if (myCon->round() == 0)
	{
		myCon->chooseHero(HERO_NAME[0], campRotate(8, 50));
		myCon->chooseHero(HERO_NAME[1], campRotate(20, 50));
		myCon->chooseHero(HERO_NAME[2], campRotate(33, 50));
		myCon->chooseHero(HERO_NAME[3], campRotate(0, 50));
	}
	else
	{
		int heroCount = 0;
		for (auto x : myCon->friendlyUnits())
			if (x->isHero())
				heroCount++;
		int newHero = heroCount % 3;
		if (newHero == 1 || newHero == 2)
			newHero = 3 - newHero;
		myCon->chooseHero(HERO_NAME[newHero]);
	}
}

void AIController::assignBaseAttack()
{
	PUnit *minBloodHero = NULL;
	UnitFilter filter;
	filter.setAreaFilter(new Circle(myBase->pos, MILITARY_BASE_RANGE));
	for (auto x : myCon->enemyUnits(filter))
		if (!minBloodHero || x->hp < minBloodHero->hp)
			minBloodHero = x;
	myCon->baseAttack(minBloodHero);
}

void AIController::action()
{
	globalStrategy();

	for (size_t i = 0; i < myHeros.size(); ++i)
		AIHeroList.push_back(make_shared<AIHero>(myHeros[i]));
	for (size_t i = 0; i < myHeros.size(); ++i)
	{
		allStrategy.push_back(new Unavalable(myHeros[i]));
		if (myHeros[i]->findBuff("Reviving") != nullptr || myHeros[i]->findBuff("Dizzy") != nullptr)
		{
			AIHeroList[i]->chooseStrategy(allStrategy);
			continue;
		}

		if(myHeros[i]->name == strScouter)
			allStrategy.push_back(new PlugEye(myHeros[i]));
		else
			for (size_t j = 0; j < enemyHeros.size(); ++j)
				allStrategy.push_back(new UseSkill(myHeros[i], enemyHeros[j]));
		for (size_t j = 0; j < enemyHeros.size(); ++j)
			allStrategy.push_back(new Attack(myHeros[i], enemyHeros[j]));
		for (size_t j = 0; j < enemyHeros.size(); ++j)
			allStrategy.push_back(new Tempt(myHeros[i], enemyHeros[j]));
		allStrategy.push_back(new GoBackHome(myHeros[i]));
		allStrategy.push_back(new CallBackHome(myHeros[i]));
		allStrategy.push_back(new GoCenterMining(myHeros[i]));
		for (size_t j = 1; j < MINE_NUM; ++j)
			allStrategy.push_back(new GoMining(myHeros[i], MINE_POS[j]));
		allStrategy.push_back(new PushBase(myHeros[i]));
		AIHeroList[i]->chooseStrategy(allStrategy);
	}
	for (size_t i = 0; i < AIHeroList.size(); ++i)
	{
		//cout << AIHeroList[i]->getStrategy()->getName() << ' ' << AIHeroList[i]->getStrategy()->getWorth() << '\t';
		AIHeroList[i]->action();
	}
}
/********************************************************************************/


/****************************** Entry Function **********************************/
void player_ai(const PMap &map, const PPlayerInfo &info, PCommand &cmd)
{
	srand((unsigned int)time(NULL));
	AIController controller(map, info, cmd);

	if (info.round == 0)
	{
		controller.buyNewHero();
		return;
	}

	controller.action();
}
/********************************************************************************/